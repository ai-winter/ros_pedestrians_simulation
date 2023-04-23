#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.2                                                                       *
*  @date     2023.04.23                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""

import xml.etree.ElementTree as ET
import sys, os
sys.path.append(os.path.abspath(os.path.join(__file__, "../../")))

from dynamic_config import XMLGenerator
from dynamic_config import PedGenerator, ObstacleGenerator

class MainGenerator(XMLGenerator):
    def __init__(self, *plugins) -> None:
        super().__init__()
        self.main_path = self.root_path + "pedestrian_simulation/"
        self.app_list = [app for app in plugins]
    
    def writeMainLaunch(self, path):
        """
        Create main launch file to run ROS node.

        Parameters
        ----------
        path: str
            the path of file(.launch.xml) to write.
        """
        launch = MainGenerator.createElement("launch")
        
        # other applications
        for app in self.app_list:
            assert isinstance(app, XMLGenerator), "Expected type of app is XMLGenerator"
            app_register = app.plugin()
            for app_element in app_register:
                launch.append(app_element)

        # include file
        include = MainGenerator.createElement("include", props={"file": "$(find ped_simulation)/launch/config.launch"})
        # robot property
        robot_cfg = self.user_cfg["robot_config"]
        include.append(MainGenerator.createElement("arg", props={"name": "model", "value": robot_cfg["robot_type"]}))
        include.append(MainGenerator.createElement("arg", props={"name": "x_pos", "value": str(robot_cfg["robot_x_pos"])}))
        include.append(MainGenerator.createElement("arg", props={"name": "y_pos", "value": str(robot_cfg["robot_y_pos"])}))
        include.append(MainGenerator.createElement("arg", props={"name": "z_pos", "value": str(robot_cfg["robot_z_pos"])}))
        include.append(MainGenerator.createElement("arg", props={"name": "yaw", "value": str(robot_cfg["robot_yaw"])}))
        include.append(MainGenerator.createElement("arg", props={"name": "map", "value": self.user_cfg["map"]}))
        include.append(MainGenerator.createElement("arg", props={"name": "rviz_file", "value": self.user_cfg["rviz_file"]}))
        include.append(MainGenerator.createElement("arg", props={"name": "world", "value": "$(arg world)"}))
        launch.append(include)
        
        MainGenerator.indent(launch)
        launch = ET.ElementTree(launch)

        with open(path, "wb+") as f:
            launch.write(f, encoding="utf-8", xml_declaration=True)

    def plugin(self):
        pass

# dynamic generator
main_gen = MainGenerator(PedGenerator(), ObstacleGenerator())
main_gen.writeMainLaunch(main_gen.main_path + "launch/main.launch")

