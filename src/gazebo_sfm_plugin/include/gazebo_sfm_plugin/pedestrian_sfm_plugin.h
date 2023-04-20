/***********************************************************
 *
 * @file: pedestrian_sfm_plugin.cpp
 * @breif: Gazebo plugin for pedestrians using social force model
 * @author: Yang Haodong
 * @update: 2023-03-15
 * @version: 1.1
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PEDESTRIAN_SFM_GAZEBO_PLUGIN_H
#define PEDESTRIAN_SFM_GAZEBO_PLUGIN_H

// C++
#include <string>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>

// Social Force Model
#include <lightsfm/sfm.hpp>

// message
#include <gazebo_sfm_plugin/ped_state.h>

namespace gazebo
{
class GZ_PLUGIN_VISIBLE PedestrianSFMPlugin : public ModelPlugin
{
public:
  /**
   * @brief Construct a gazebo plugin
   */
  PedestrianSFMPlugin();

  /**
   * @brief De-Construct a gazebo plugin
   */
  ~PedestrianSFMPlugin();

  /**
   * @brief Load the actor plugin.
   * @param _model  Pointer to the parent model.
   * @param _sdf    Pointer to the plugin's SDF elements.
   */
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /**
   * @brief Initialize the social force model.
   */
  virtual void Reset();

protected:
private:
  bool pose_init_;  // initialized

  sdf::ElementPtr sdf_;      // Pointer to the sdf element.
  physics::ActorPtr actor_;  // Pointer to the parent actor.
  physics::WorldPtr world_;  // Pointer to the world, for convenience.

  std::unique_ptr<ros::NodeHandle> node_;  // Gazebo ROS node
  ros::Publisher pose_pub_;                // pose publisher
  ros::Publisher vel_pub_;                 // velocity publisher
  ros::ServiceServer state_server_;        // pedestrian state server

  std::vector<event::ConnectionPtr> connections_;  // List of connections

  sfm::Agent sfm_actor_;                        // this actor as a SFM agent
  physics::TrajectoryInfoPtr trajectory_info_;  // Custom trajectory info.

  double animation_factor_ = 4.5;  // Time scaling factor.
  double people_dist_;             // Maximum distance to detect nearby pedestrians.

  std::vector<std::string> group_names_;    // names of the other models in my walking group.
  std::vector<std::string> ignore_models_;  // List of models to ignore. Used for vector field
  std::vector<sfm::Agent> other_actors_;    // vector of pedestrians detected.

  double last_pose_x_, last_pose_y_;       // last pose
  double px_, py_, pz_, vx_, vy_, theta_;  // current state
  common::Time last_update_;               // Time of the last update.
  ignition::math::Vector3d velocity_;      // Velocity of the actor

  /**
   * @brief Function that is called every update cycle.
   * @param _info Timing information.
   */
  void OnUpdate(const common::UpdateInfo& _info);

  bool OnStateCallBack(gazebo_sfm_plugin::ped_state::Request& req, gazebo_sfm_plugin::ped_state::Response& resp);

  /**
   * @brief Helper function to detect the closest obstacles.
   */
  void handleObstacles();

  /**
   * @brief Helper function to detect the nearby pedestrians (other actors).
   */
  void handlePedestrians();
};
}  // namespace gazebo
#endif
