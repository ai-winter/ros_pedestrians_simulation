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
#include <functional>
#include <stdio.h>
#include <string>

#include <pedestrian_visual_plugin.h>


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedestrianVisualPlugin)

/**
 * @brief Construct a gazebo plugin
 */
PedestrianVisualPlugin::PedestrianVisualPlugin()
{
}

/**
 * @brief De-Construct a gazebo plugin
 */
PedestrianVisualPlugin::~PedestrianVisualPlugin()
{
  ped_visual_pub_.shutdown();
}


/**
 * @brief Load the actor plugin.
 * @param _model  Pointer to the parent model.
 * @param _sdf    Pointer to the plugin's SDF elements.
 */
void PedestrianVisualPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // gazebo model pointer
  sdf_ = _sdf;
  actor_ = boost::dynamic_pointer_cast<physics::Actor>(_model);
  world_ = actor_->GetWorld();

  // Create the ROS node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }
  node_.reset(new ros::NodeHandle("gazebo_client"));

  // topic publisher
	ped_visual_pub_ = node_->advertise<pedsim_msgs::TrackedPersons>("/ped_visualization", 1);
	states_client_ = node_->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // Bind the update callback function
  connections_.push_back(
      event::Events::ConnectWorldUpdateBegin(std::bind(&PedestrianVisualPlugin::OnUpdate, this, std::placeholders::_1)));


}

/**
 * @brief Publish pedestrians visualization information
 */
void PedestrianVisualPlugin::publishPedVisuals()
{
	int human_id = 0;

  pedsim_msgs::TrackedPersons tracked_people;
	std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = "map";
  tracked_people.header = msg_header;

	for (unsigned int i = 0; i < world_->ModelCount(); ++i)
  {
    physics::ModelPtr model = world_->ModelByIndex(i);
    if (((int)model->GetType() == (int)actor_->GetType()))
		{
			human_id++;
			pedsim_msgs::TrackedPerson person;
			person.track_id = human_id;
			person.is_occluded = false;
			person.detection_id = human_id;

			gazebo_msgs::GetModelState model_state;
			model_state.request.model_name = model->GetName();
			model_state.request.relative_entity_name = "world";
			states_client_.call(model_state);

			ignition::math::Pose3d pose = model->WorldPose();
			ignition::math::Vector3d rpy = pose.Rot().Euler();
			const double theta = rpy.Z();
			geometry_msgs::PoseWithCovariance pose_with_cov;
			pose_with_cov.pose = model_state.response.pose;
			tf2::Quaternion q;
			q.setRPY(0, 0, theta - 1.57);
			tf2::convert(q, pose_with_cov.pose.orientation);
			person.pose = pose_with_cov;

			geometry_msgs::TwistWithCovariance twist_with_cov;
			twist_with_cov.twist = model_state.response.twist;
			person.twist = twist_with_cov;

    	tracked_people.tracks.push_back(person);
		}
  }
	ped_visual_pub_.publish(tracked_people);
}

/**
 * @brief Function that is called every update cycle.
 * @param _info Timing information.
 */
void PedestrianVisualPlugin::OnUpdate(const common::UpdateInfo& _info)
{
	publishPedVisuals();
}

