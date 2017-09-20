/*
 * mavros_auto.cpp
 *
 *  Created on: Sep 20, 2017
 *      Author: lyx
 */

#include "mavros_auto.h"

extern mavros_auto  ros_d;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped msg);
void global_cb(const mavros_msgs::GlobalPositionTarget msg);




mavros_auto::mavros_auto()
{
	// TODO Auto-generated constructor stub
	state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
	global_position_sub = nh.subscribe<mavros_msgs::GlobalPositionTarget>("mavros/global_position/global", 10, global_cb);
	local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}


mavros_auto::~mavros_auto()
{
	// TODO Auto-generated destructor stub
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	ros_d.current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped msg)
{
	ros_d.now_pos.x = msg.pose.position.x;
	ros_d.now_pos.y = msg.pose.position.y;
	ros_d.now_pos.z = msg.pose.position.z;
}

void global_cb(const mavros_msgs::GlobalPositionTarget msg)
{
	ros_d.now_global_pos.latitude = msg.latitude;
	ros_d.now_global_pos.longitude = msg.longitude;
	ros_d.now_global_pos.altitude = msg.longitude;
}

bool mavros_auto::waypointPusher(mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, int frame, int command,
		bool isCurrent, bool autoCont, float param1, float param2, float param3, float param4, float lat, float lon, float alt)
{
	client = node.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

	mavros_msgs::Waypoint nextWaypoint;

	nextWaypoint.frame = frame;
	nextWaypoint.command = command;
	nextWaypoint.is_current = isCurrent;
	nextWaypoint.autocontinue = autoCont;
	nextWaypoint.param1 = param1;
	nextWaypoint.param2 = param2;
	nextWaypoint.param3 = param3;
	nextWaypoint.param4 = param4;
	nextWaypoint.x_lat = lat;
	nextWaypoint.y_long = lon;
	nextWaypoint.z_alt = alt;

	pusher.request.waypoints.push_back(nextWaypoint);

	if (client.call(pusher))
		ROS_INFO_STREAM("PUSHED WAYPOINT");
	else
		ROS_INFO_STREAM("PUSH FAILED");

	return client.call(pusher);
}
bool mavros_auto::preparation(void)
{
	bool data;
	if (ros_d.now_global_pos.latitude != 0 || ros_d.now_global_pos.longitude != 0 ||ros_d.current_state.connected==1)
		data=1;
	else
		data=0;

	return data;
}

bool mavros_auto::arm_copter(void)
{
	bool data;
	mavros_msgs::SetMode offb_set_mode;
	mavros_msgs::CommandBool arm_cmd;

	if (current_state.mode != "guided" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
	{
		ROS_INFO("Offboard enabled");
	}

	if (!current_state.armed && arming_client.call(arm_cmd) && arm_cmd.response.success)
	{
		ROS_INFO("Vehicle armed");
		data=1;
	}else
	{
		data=0;
	}
	return data;

}


