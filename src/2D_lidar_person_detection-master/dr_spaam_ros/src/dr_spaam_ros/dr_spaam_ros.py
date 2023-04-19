# import time
import numpy as np
import math
import rospy
import tf, tf2_ros

from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs.tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker

from kalman_filter import Kalman

from dr_spaam.detector import Detector


class PersonEstimate:
    def __init__(self) -> None:
        self.filter = Kalman()
        self.last_pos = None
        self.age = None
    
    def update(self, time_stamp, new_pos):
        dp = new_pos - self.last_pos
        time = (time_stamp - self.age).to_sec()
        vx, vy = dp[0] / time, dp[1] / time

        # update
        self.last_pos = new_pos
        self.age = time_stamp
        self.filter.update([vx, vy])
    
    @property
    def velocity(self):
        k = self.filter.values()
        if k is None:
            return None
        return (k[0], k[1])


class DrSpaamROS:
    """ROS node to detect pedestrian using DROW3 or DR-SPAAM."""

    def __init__(self):
        self._read_params()
        self._detector = Detector(
            self.weight_file,
            model=self.detector_model,
            gpu=True,
            stride=self.stride,
            panoramic_scan=self.panoramic_scan,
        )
        self._init()

    def _read_params(self):
        """
        @brief      Reads parameters from ROS server.
        """
        self.weight_file = rospy.get_param("~weight_file")
        self.conf_thresh = rospy.get_param("~conf_thresh")
        self.stride = rospy.get_param("~stride")
        self.detector_model = rospy.get_param("~detector_model")
        self.panoramic_scan = rospy.get_param("~panoramic_scan")

    def _init(self):
        """
        @brief      Initialize ROS connection.
        """
        self.last_pos = None
        self.last_update = None
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)


        self.people = []
        # Publisher
        self._dets_pub = rospy.Publisher("/human_track_detections", PoseArray, queue_size=1)

        topic, queue_size, latch = read_publisher_param("rviz")
        self._rviz_pub = rospy.Publisher(
            topic, Marker, queue_size=queue_size, latch=latch
        )

        # Subscriber
        topic, queue_size = read_subscriber_param("scan")
        self._scan_sub = rospy.Subscriber(
            topic, LaserScan, self._scan_callback, queue_size=queue_size
        )

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

        # t = time.time()
        dets_xy, dets_cls, _ = self._detector(scan)
        # print("[DrSpaamROS] End-to-end inference time: %f" % (t - time.time()))

        # confidence threshold
        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]
        # print(dets_xy, type(dets_xy))

        # convert to ros msg and publish
        self.publishPoseArray(msg, dets_xy)

        rviz_msg = self.detections_to_rviz_marker(dets_xy, dets_cls)
        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)


    def detections_to_rviz_marker(self, dets_xy, dets_cls):
        """
        @brief     Convert detection to RViz marker msg. Each detection is marked as
                a circle approximated by line segments.
        """
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = "dr_spaam_ros"
        msg.id = 0
        msg.type = Marker.LINE_LIST

        # set quaternion so that RViz does not give warning
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.scale.x = 0.03  # line width
        # red color
        msg.color.g = 1.0
        msg.color.a = 1.0

        # circle
        r = 0.4
        ang = np.linspace(0, 2 * np.pi, 20)
        xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

        # to msg
        for d_xy, _ in zip(dets_xy, dets_cls):
            for i in range(len(xy_offsets) - 1):
                # start point of a segment
                p0 = Point()
                p0.x = -d_xy[0] + xy_offsets[i, 0]
                p0.y = -d_xy[1] + xy_offsets[i, 1]
                p0.z = 0.0
                msg.points.append(p0)

                # end point
                p1 = Point()
                p1.x = -d_xy[0] + xy_offsets[i + 1, 0]
                p1.y = -d_xy[1] + xy_offsets[i + 1, 1]
                p1.z = 0.0
                msg.points.append(p1)

        return msg


    # def publishPoseArray(self, msg, dets_xy):
    #     for i, person in enumerate(self.people):
    #         # out of date
    #         if rospy.Time.now() - person.age > rospy.Duration(2.0):
    #             self.people.pop(i)
    #             del person

    #     pose_array = PoseArray()
    #     for d_xy in dets_xy:
    #         map_pos = np.array(self._calMapPos(d_xy))
    #         new_person = True

    #         for person in self.people:
    #             delta = np.sqrt(np.sum((person.last_pos - map_pos)**2))
    #             if delta < 0.4:
    #                 person.update(msg.header.stamp, map_pos)
    #                 new_person = False

    #                 p = Pose()
    #                 p.position.x = map_pos[0]
    #                 p.position.y = map_pos[1]
    #                 p.position.z = 0.0
    #                 print(person.velocity)
    #                 yaw = math.atan2(person.velocity[0], person.velocity[1])
    #                 q = tf.transformations.quaternion_from_euler(0, 0, np.pi / 2 + yaw)
    #                 p.orientation.x = q[0]
    #                 p.orientation.y = q[1]
    #                 p.orientation.z = q[2]
    #                 p.orientation.w = q[3]
    #                 pose_array.poses.append(p)

    #                 break
            
    #         if new_person:
    #             person = PersonEstimate()
    #             person.last_pos = map_pos
    #             person.age = msg.header.stamp
    #             self.people.append(person)
    #     print(len(self.people))
    #     pose_array.header.stamp = msg.header.stamp
    #     pose_array.header.frame_id = "map"
    #     self._dets_pub.publish(pose_array)


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

        # print("time: ", rospy.Time.now())
        # print("last: ", self.last_pos)
        # print("now: ", dets_xy)

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


            # print("dist: ", dist)
            # print("min dist: ", min_dist)
            # print("arg: ", arg_min_dist)

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
                print("x:", d_xy[0], "--------y: ", d_xy[1], "------dt:", dt)
                # print("x: ", dp[0] / dt, "y: ", dp[1] / dt)
                yaw = math.atan2(dp[0], dp[1])
                # print(i, "------", d_xy, "----", self.last_pos[arg_min_dist[i], :], "----", yaw)
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

        pos_map = self.tf_buffer.transform(pos_scan, "map")
        return (pos_map.point.x, pos_map.point.y)


def read_subscriber_param(name):
    """
    @brief      Convenience function to read subscriber parameter.
    """
    topic = rospy.get_param("~subscriber/" + name + "/topic")
    queue_size = rospy.get_param("~subscriber/" + name + "/queue_size")
    return topic, queue_size


def read_publisher_param(name):
    """
    @brief      Convenience function to read publisher parameter.
    """
    topic = rospy.get_param("~publisher/" + name + "/topic")
    queue_size = rospy.get_param("~publisher/" + name + "/queue_size")
    latch = rospy.get_param("~publisher/" + name + "/latch")
    return topic, queue_size, latch
