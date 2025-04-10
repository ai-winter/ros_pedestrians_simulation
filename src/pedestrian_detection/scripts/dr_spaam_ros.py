#!/usr/bin/env python

# import time
import numpy as np
import math
import rospy
import tf, tf2_ros

from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from dr_spaam.detector import Detector


class DrSpaamROS:
    """ROS node to detect pedestrian using DROW3 or DR-SPAAM."""

    def __init__(self):
        self.weight = rospy.get_param("~weight")
        self.detector_model = rospy.get_param("~model")
        self.conf_thresh = .5
        # use this to skip laser points
        self.stride = 1
        # Set to true if the scan covers 360 degree
        self.panoramic_scan = True

        self._detector = Detector(
            self.weight,
            model=self.detector_model,
            gpu=True,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )

        self.last_pos = None
        self.last_update = None
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Publisher
        self._dets_pub = rospy.Publisher("/human_track_detections", PoseArray, queue_size=1)
        self._rviz_pub = rospy.Publisher("/human_track_marker", Marker, queue_size=1)

        # Subscriber
        self._scan_sub = rospy.Subscriber("scan", LaserScan, self._scan_callback, queue_size=1)       

    def _scan_callback(self, msg):
        if (
            self._dets_pub.get_num_connections() == 0
            and self._rviz_pub.get_num_connections() == 0
        ):
            return

        # TODO check the computation here
        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges))
            )

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        dets_xy, dets_cls, _ = self._detector(scan)

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        # convert to ros msg and publish
        self.publishPoseArray(msg, dets_xy)
        self.publishPosMarker(msg, dets_xy)

    def publishPosMarker(self, msg, dets_xy):
        rviz_msg = Marker()
        rviz_msg.action = Marker.ADD
        rviz_msg.ns = "ped_tracker_ros"
        rviz_msg.id = 0
        rviz_msg.type = Marker.LINE_LIST

        # set quaternion so that RViz does not give warning
        rviz_msg.pose.orientation.x = 0.0
        rviz_msg.pose.orientation.y = 0.0
        rviz_msg.pose.orientation.z = 0.0
        rviz_msg.pose.orientation.w = 1.0

        rviz_msg.scale.x = 0.03  # line width
        rviz_msg.color.g = 1.0
        rviz_msg.color.a = 1.0

        # circle
        r = 0.4
        ang = np.linspace(0, 2 * np.pi, 20)
        xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

        # to msg
        for d_xy in dets_xy:
            for i in range(len(xy_offsets) - 1):
                # start point of a segment
                p0 = Point()
                p0.x = -d_xy[0] + xy_offsets[i, 0]
                p0.y = -d_xy[1] + xy_offsets[i, 1]
                p0.z = 0.0
                rviz_msg.points.append(p0)

                # end point
                p1 = Point()
                p1.x = -d_xy[0] + xy_offsets[i + 1, 0]
                p1.y = -d_xy[1] + xy_offsets[i + 1, 1]
                p1.z = 0.0
                rviz_msg.points.append(p1)

        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)

    def publishPoseArray(self, msg, dets_xy):
        if self.last_update is None:
            self.last_update = msg.header.stamp

        # transform from scan frame to map frame
        if dets_xy.size:
            map_pos = []
            for d_xy in dets_xy:
                single_map_pos = self._calMapPos(d_xy)
                map_pos.append((single_map_pos[0], single_map_pos[1]))
            dets_xy = np.array(map_pos)

        if self.last_pos is None or not self.last_pos.size or not dets_xy.size:
            self.last_pos = dets_xy
        else:
            now2 = (dets_xy ** 2).sum(axis=1).reshape((-1, 1))
            last2 = (self.last_pos ** 2).sum(axis=1)
            dist = now2 + last2 - 2 * dets_xy @ self.last_pos.T
            dist[dist < 0] = 0.0
            # ------------------last
            # | dij
            # |
            # now 
            dist = np.sqrt(dist)
            min_dist = np.min(dist, axis=1)
            arg_min_dist = np.argmin(dist, axis=1)

            pose_array = PoseArray()
            for i, d_xy in enumerate(dets_xy):
                # TODO: fixed threshold to auto-threshold
                if min_dist[i] > 0.35:
                    continue

                p = Pose()
                p.position.x = d_xy[0]
                p.position.y = d_xy[1]
                p.position.z = 0.0
                # estimate velocity
                dp = d_xy - self.last_pos[arg_min_dist[i], :]
                dt = (msg.header.stamp - self.last_update).to_sec()
                # print("x:", d_xy[0], "--------y: ", d_xy[1], "------dt:", dt)

                yaw = math.atan2(dp[0], dp[1])
                q = tf.transformations.quaternion_from_euler(0, 0, np.pi / 2 + yaw)
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]

                pose_array.poses.append(p)
            
            # pose_array.header = msg.header
            pose_array.header.stamp = msg.header.stamp
            pose_array.header.frame_id = "map"

            self.last_pos = dets_xy
            self._dets_pub.publish(pose_array)
        self.last_update = msg.header.stamp

    def _calMapPos(self, d_xy):
        pos_scan = PointStamped()
        pos_scan.header.stamp = rospy.Time()
        pos_scan.header.frame_id = "base_scan"
        pos_scan.point.x = -d_xy[0]
        pos_scan.point.y = -d_xy[1]
        pos_scan.point.z = 0.0

        pos_map = self.tf_buffer.transform(pos_scan, "map", timeout=rospy.Duration(0.5))
        return (pos_map.point.x, pos_map.point.y)


if __name__ == '__main__':
    rospy.init_node('dr_spaam_ros')
    try:
        DrSpaamROS()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()