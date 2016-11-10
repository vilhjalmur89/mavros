/**
 * @brief AvoidanceTriplet plugin
 * @file avoidance_triplet.cpp
 * @author Vilhjalmur Vilhjalmsson <villi@px4.io>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2016 Vilhjalmur Vilhjalmsson <villi@px4.io>
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/AvoidanceTriplet.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief AvoidanceTriplet plugin
 *
 * Sends avoidance triplet to FCU controller.
 */
class AvoidanceTripletPlugin : public plugin::PluginBase {
public:
	AvoidanceTripletPlugin() : PluginBase(),
		nh("~")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		
		avoidance_triplet_sub = nh.subscribe("avoidance_triplet", 
			10, &AvoidanceTripletPlugin::avoidance_triplet_cb, this);

		planning_triplet_pub = nh.advertise<mavros_msgs::AvoidanceTriplet>("planning_triplet", 10);
	}

	Subscriptions get_subscriptions()
	{
		// return { /* Rx disabled */ };
		return {
			make_handler(&AvoidanceTripletPlugin::handle_avoidance_triplet),
		};
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber avoidance_triplet_sub;
	ros::Publisher planning_triplet_pub;


	Eigen::Vector3d transformToNed(const geometry_msgs::Point &point) {
		Eigen::Vector3d vec;
		tf::pointMsgToEigen(point, vec);
		return ftf::transform_frame_enu_ned(vec);
	}	

	geometry_msgs::Point vecToPoint(const Eigen::Vector3d &vec) {
		geometry_msgs::Point point;
		point.x = vec.x();
		point.y = vec.y();
		point.z = vec.z();
		return point;
	}


	/* -*- callbacks -*- */
	
	// Receive message from Ros, forward it to FCU
	void avoidance_triplet_cb(const mavros_msgs::AvoidanceTriplet::ConstPtr &msg) {
		mavlink::common::msg::AVOIDANCE_TRIPLET triplet{};

		// The message is in ENU, so we transform it to NED
		Eigen::Vector3d prev = transformToNed(msg->prev);
		Eigen::Vector3d ctrl = transformToNed(msg->ctrl);
		Eigen::Vector3d next = transformToNed(msg->next);

		// Fill in the Mavlink message
		triplet.time_usec = ros::Time::now().toNSec() / 1000;
		m_uas->msg_set_target(triplet);
		triplet.prev_x = prev.x();
		triplet.prev_y = prev.y();
		triplet.prev_z = prev.z();
		triplet.ctrl_x = ctrl.x();
		triplet.ctrl_y = ctrl.y();
		triplet.ctrl_z = ctrl.z();
		triplet.next_x = next.x();
		triplet.next_y = next.y();
		triplet.next_z = next.z();
		triplet.duration = msg->duration;
		triplet.max_acc = msg->max_acc;
		triplet.acc_per_err = msg->acc_per_err;
		printf("%d %2.2f %2.2f %2.2f \n", triplet.MSG_ID, triplet.prev_x, triplet.prev_y, triplet.prev_z);

		// Send the message to FCU
		UAS_FCU(m_uas)->send_message_ignore_drop(triplet);
	}

	// Receive message front()m FCU, forward it to ROS
	void handle_avoidance_triplet(const mavlink::mavlink_message_t *msg, 
								  mavlink::common::msg::AVOIDANCE_TRIPLET &triplet) {

		// FCU is in NED, need to transform to ENU
		Eigen::Vector3d prev(triplet.prev_x, triplet.prev_y, triplet.prev_z);
		Eigen::Vector3d ctrl(triplet.ctrl_x, triplet.ctrl_y, triplet.ctrl_z);
		Eigen::Vector3d next(triplet.next_x, triplet.next_y, triplet.next_z);
		prev = ftf::transform_frame_ned_enu(prev);
		ctrl = ftf::transform_frame_ned_enu(ctrl);
		next = ftf::transform_frame_ned_enu(next);

		// Fill in the Mavros message
		mavros_msgs::AvoidanceTriplet ros_msg;
		ros_msg.header = m_uas->synchronized_header("local_origin", triplet.time_usec);
		ros_msg.prev = vecToPoint(prev);
		ros_msg.ctrl = vecToPoint(ctrl);
		ros_msg.next = vecToPoint(next);
		ros_msg.duration = triplet.duration;
		ros_msg.max_acc = triplet.max_acc;
		ros_msg.acc_per_err = triplet.acc_per_err;

		// Publish as Ros-topic
		planning_triplet_pub.publish(ros_msg);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AvoidanceTripletPlugin, mavros::plugin::PluginBase)
