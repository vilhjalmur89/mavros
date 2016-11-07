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
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle nh;
	ros::Subscriber avoidance_triplet_sub;

	/* -*- callbacks -*- */

	void avoidance_triplet_cb(const mavros_msgs::AvoidanceTriplet::ConstPtr &msg) {
		mavlink::common::msg::AVOIDANCE_TRIPLET triplet{};
		triplet.time_usec = ros::Time::now().toNSec() / 1000;
		triplet.prev_x = msg.prev.x;
		triplet.prev_y = msg.prev.y;
		triplet.prev_z = msg.prev.z;
		triplet.ctrl_x = msg.ctrl.x;
		triplet.ctrl_y = msg.ctrl.y;
		triplet.ctrl_z = msg.ctrl.z;
		triplet.next_x = msg.next.x;
		triplet.next_y = msg.next.y;
		triplet.next_z = msg.next.z;
		triplet.duration = msg.duration;
		triplet.max_acc = msg.max_acc;
		triplet.acc_per_err = msg.acc_per_err;

		UAS_FCU(m_uas)->send_message_ignore_drop(triplet);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AvoidanceTripletPlugin, mavros::plugin::PluginBase)
