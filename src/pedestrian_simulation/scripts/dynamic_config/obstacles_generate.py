#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.2                                                                       *
*  @date     2023.03.15                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
import rospy, tf
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Quaternion, Pose, Point
import xml.etree.ElementTree as ET
from .xml_generate import XMLGenerator


class ObstacleGenerator(XMLGenerator):
    def __init__(self) -> None:
        super().__init__()
        if "obstacles" in self.user_cfg.keys() and self.user_cfg["obstacles"]:
            self.obs_cfg = ObstacleGenerator.yamlParser(self.root_path + "user_config/" + self.user_cfg["obstacles"])
        else:
            self.obs_cfg = None
        
        self.box_obs, self.cylinder_obs, self.circle_obs = [], [], []
    
    def plugin(self):
        """
        Implement of obstacles application.
        """
        app_register = []
        if not self.obs_cfg is None:
            '''app register'''
            # obstacles generation
            obs_gen = ObstacleGenerator.createElement("node", props={"pkg": "ped_simulation", "type": "obstacles_genertate_ros.py",
                "name": "obstacles_generate", "args": "user_config.yaml","output": "screen"})
            app_register.append(obs_gen)
        
        return app_register

    def spawn(self):
        def spawnObstacle(proxy, name, pose, model_type, **kwargs):
            if model_type == "BOX":
                assert "m" in kwargs and \
                    "w" in kwargs and \
                    "d" in kwargs and \
                    "h" in kwargs and \
                    "c" in kwargs,    \
                    "Parameters of {} are `m`, `w`, `d`, `h`, `c` which mean mass, \
                        width, depth, height and color, ".format(model_type)
                urdf = self.constructBoxURDF(kwargs["m"], kwargs["w"], kwargs["d"], kwargs["h"], kwargs["c"])
            else:
                raise NotImplementedError

            proxy(name, urdf, "", pose, "world")


        rospy.wait_for_service("gazebo/spawn_urdf_model")
        proxy = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)

        for obs in self.obs_cfg["obstacles"]:
            pose = [float(i) for i in obs["pose"].split()]
            x, y, z, r, p, yaw = pose
            orient = tf.transformations.quaternion_from_euler(r, p, yaw)
            orient = Quaternion(orient[0], orient[1], orient[2], orient[3])

            if obs["type"] == "BOX":
                z = z if z else obs["props"]["h"] / 2
                pose = Pose(Point(x=x, y=y, z=z), orient)
                params = obs["props"]
                params["c"] = obs["color"]
                spawnObstacle(proxy, obs["type"] + str(len(self.box_obs)), pose, obs["type"], **params)
                self.box_obs.append(obs)

    def constructBoxURDF(self, m, w, d, h, c):
        ixx = (m / 12.0) * (pow(d, 2) + pow(h, 2))
        iyy = (m / 12.0) * (pow(w, 2) + pow(h, 2))
        izz = (m / 12.0) * (pow(w, 2) + pow(d, 2))

        static_obs = ObstacleGenerator.createElement("robot", props={"name": "box"})
        link = ObstacleGenerator.createElement("link", props={"name": "box_link"})

        inertial = ObstacleGenerator.createElement("inertial")
        inertial.append(ObstacleGenerator.createElement("origin", props={"xyz": "0 0 0", "rpy": "0 0 0"}))
        inertial.append(ObstacleGenerator.createElement("mass", props={"value": "1.0"}))
        inertial.append(ObstacleGenerator.createElement("inertia", props={"ixx": str(ixx), "ixy": "0.0", "ixz": "0.0",
            "iyy": str(iyy), "iyz": "0.0", "izz": str(izz)}))

        collision = ObstacleGenerator.createElement("collision")
        collision.append(ObstacleGenerator.createElement("origin", props={"xyz": "0 0 0", "rpy": "0 0 0"}))
        geometry = ObstacleGenerator.createElement("geometry")
        geometry.append(ObstacleGenerator.createElement("box", props={"size": "{:f} {:f} {:f}".format(w, d, h)}))
        collision.append(geometry)

        visual = ObstacleGenerator.createElement("visual")
        visual.append(ObstacleGenerator.createElement("origin", props={"xyz": "0 0 0", "rpy": "0 0 0"}))
        visual.append(geometry)
        r, g, b, a = ObstacleGenerator.color(c)
        visual.append(ObstacleGenerator.createElement("color", props={"rgba": "{:f} {:f} {:f} {:f}".format(r, g, b, a)}))

        link.append(inertial)
        link.append(collision)
        link.append(visual)

        gazebo = ObstacleGenerator.createElement("gazebo", props={"reference": "box_link"})
        gazebo.append(ObstacleGenerator.createElement("kp", text=str(100000.0)))
        gazebo.append(ObstacleGenerator.createElement("kd", text=str(100000.0)))
        gazebo.append(ObstacleGenerator.createElement("mu1", text=str(10.0)))
        gazebo.append(ObstacleGenerator.createElement("mu2", text=str(10.0)))
        gazebo.append(ObstacleGenerator.createElement("material", text="Gazebo/" + c))

        static_obs.append(link)
        static_obs.append(gazebo)

        ObstacleGenerator.indent(static_obs)

        return ET.tostring(static_obs).decode()


    @staticmethod
    def color(color_name):
        if color_name == "Blue":
            return (0, 0, 0.8, 1)
        elif color_name == "Red":
            return (0.8, 0, 0, 1)
        elif color_name == "Green":
            return (0, 0.8, 0, 1)
        elif color_name == "Grey":
            return (0.75, 0.75, 0.75, 1)
        elif color_name == "White":
            return (1.0, 1.0, 1.0, 1)
        elif color_name == "Black":
            return (0, 0, 0, 1)
        else:
            return (0.75, 0.75, 0.75, 1)
