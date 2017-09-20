/*
 * mavros_auto.h
 *
 *  Created on: Sep 20, 2017
 *      Author: lyx
 */

#ifndef MAVROS_AUTO_H_
#define MAVROS_AUTO_H_

#include <string>
#include <vector>
#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#define FRAME 3

class mavros_auto
{
public:
	mavros_auto();
//	virtual ~mavros_auto();

	mavros_msgs::State current_state;
	geometry_msgs::Point now_pos;
	mavros_msgs::GlobalPositionTarget now_global_pos;

	ros::NodeHandle nh;

	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;

	ros::Subscriber state_sub;
	ros::Subscriber position_sub;
	ros::Subscriber global_position_sub;
	ros::Publisher local_pos_pub;
	ros::ServiceClient arming_client;
	ros::ServiceClient takeoff_client;
	ros::ServiceClient set_mode_client;

	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void pose_cb(const geometry_msgs::PoseStamped msg);
	void global_cb(const mavros_msgs::GlobalPositionTarget msg);
	bool waypointPusher(mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, int frame, int command,
			bool isCurrent, bool autoCont, float param1, float param2, float param3, float param4, float lat, float lon, float alt);

};

#endif /* MAVROS_AUTO_H_ */
