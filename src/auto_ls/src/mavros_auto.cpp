/*
 * mavros_auto.cpp
 *
 *  Created on: Sep 20, 2017
 *      Author: lyx
 */

#include "mavros_auto.h"

mavros_auto::mavros_auto()
{
	// TODO Auto-generated constructor stub
	this->state_sub = this->nh.subscribe<mavros_msgs::State>("mavros/state", 10, &mavros_auto::state_cb, this);
	this->position_sub = this->nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &mavros_auto::pose_cb, this);
	this->cur_point_pub =this->nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 10,&mavros_auto::cur_point_cb,this);
	this->global_position_sub = this->nh.subscribe<mavros_msgs::GlobalPositionTarget>("mavros/global_position/global", 10,
			&mavros_auto::global_cb, this);
	this->home_point_sub = this->nh.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 10, &mavros_auto::home_cb, this);

	local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

mavros_auto::~mavros_auto()
{
	// TODO Auto-generated destructor stub
}

void mavros_auto::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void mavros_auto::pose_cb(const geometry_msgs::PoseStamped msg)
{
	now_pos.x = msg.pose.position.x;
	now_pos.y = msg.pose.position.y;
	now_pos.z = msg.pose.position.z;
}

void mavros_auto::global_cb(const mavros_msgs::GlobalPositionTarget msg)
{
	now_global_pos.latitude = msg.latitude;
	now_global_pos.longitude = msg.longitude;
	now_global_pos.altitude = msg.longitude;

}
void mavros_auto::home_cb(const mavros_msgs::HomePosition msg)
{
	home_pos.latitude = msg.latitude;
	home_pos.longitude = msg.longitude;
	home_pos.altitude = msg.altitude;
}
void mavros_auto::cur_point_cb(const mavros_msgs::WaypointList msg)
{

	int length,i;//max 2506
	length=curPoint.waypoints.size();
	i=0;
	curPoint.waypoints=msg.waypoints;
	for(i;i<length;i++)
	{
		if(curPoint.waypoints[i].is_current==true)
			break;
	}
	way.current=i;
	way.max=curPoint.waypoints.size();
}
mavros_auto::lat_lon_alt_t mavros_auto::cal_pos(double x, double y, double z)
{
	lat_lon_alt_t pos;

	return pos;
}
bool mavros_auto::waypointPusher(int frame, int command, bool isCurrent, bool autoCont, float param1, float param2, float param3,
		float param4, float lat, float lon, float alt)
{
	pushClient = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

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

	wayPusher.request.waypoints.push_back(nextWaypoint);

	if (pushClient.call(wayPusher))
		ROS_INFO_STREAM("PUSHED WAYPOINT");
	else
		ROS_INFO_STREAM("PUSH FAILED");

	return pushClient.call(wayPusher);
}
bool mavros_auto::preparation(void)
{
	bool data;
	if (current_state.connected == 1)
	{
		if (home_pos.latitude != 0 || home_pos.longitude != 0)
			data = 1;
		else
			data = 0;
	}
	else
		data = 0;

	return data;
}

bool mavros_auto::arm_copter(void)
{
	bool data=0;
	mavros_msgs::SetMode offb_set_mode;
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	offb_set_mode.request.custom_mode = "guided";

	if (current_state.mode != "guided" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
	{
		ROS_INFO("guided enabled");
	}

	if (!current_state.armed && arming_client.call(arm_cmd) && arm_cmd.response.success)
	{
		ROS_INFO("Vehicle armed");
		data = 1;
	}

	return data;

}

bool mavros_auto::set_auto(void)
{
	bool data = 0;
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "auto";
	if (current_state.mode != "auto" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
	{
		ROS_INFO("auto enabled");
		data = 1;
	}
	return data;

}

