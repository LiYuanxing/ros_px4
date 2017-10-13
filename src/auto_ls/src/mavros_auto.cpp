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
	this->cur_point_pub = this->nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 10, &mavros_auto::cur_point_cb, this);
	this->global_position_sub = this->nh.subscribe<mavros_msgs::GlobalPositionTarget>("mavros/global_position/global", 10,
			&mavros_auto::global_cb, this);
	this->home_point_sub = this->nh.subscribe<mavros_msgs::HomePosition>("mavros/home_position/home", 10, &mavros_auto::home_cb, this);

	local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	waypointpush_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
	set_current_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("mavros/mission/set_current");
}

mavros_auto::~mavros_auto()
{
	// TODO Auto-generated destructor stub
}

void mavros_auto::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void mavros_auto::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	now_pos.x = msg->pose.position.x;
	now_pos.y = msg->pose.position.y;
	now_pos.z = msg->pose.position.z;
}

void mavros_auto::global_cb(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg)
{
	now_global_pos.latitude =  msg->latitude;
	now_global_pos.longitude = msg->longitude;
	now_global_pos.altitude =  msg->longitude;

}
void mavros_auto::home_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
	home_pos.latitude =  msg->latitude;
	home_pos.longitude = msg->longitude;
	home_pos.altitude =  msg->altitude;
}
void mavros_auto::cur_point_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{

	int length, i; //max 718
	length = curPoint.waypoints.size();
	i = 0;
	curPoint.waypoints = msg->waypoints;
	for (i; i < length; i++)
	{
		if (curPoint.waypoints[i].is_current == true)
			break;
	}

	way.current = i;
	way.max=curPoint.waypoints.size();

	//ROS_INFO("cur:%d max:%d ",way.current,way.max);
}

bool mavros_auto::waypointPusher(int frame, int command, bool isCurrent, bool autoCont, float param1, float param2, float param3,
		float param4, float lat, float lon, float alt)
{
	mavros_msgs::Waypoint nextWaypoint;
	bool result;

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
	result =waypointpush_client.call(wayPusher);
	return result;
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
	bool data = 0;
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
bool mavros_auto::mission_clear(void)
{
	wayPusher.request.waypoints.clear();
	return true;
}
bool mavros_auto::mission_set_current(int num)
{
	mavros_msgs::WaypointSetCurrent current;
	current.request.wp_seq = num;

	if (set_current_client.call(current))
		ROS_INFO_STREAM("set ok");
	else
		ROS_INFO_STREAM("set fail");

	return set_current_client.call(current);
}

bool mavros_auto::mission_home_takeoff(void)
{
	waypointPusher(LS_GLOBAL_REL_ALT, TAKE_OFF, false, false, 0, 0, 0, 0, home_pos.latitude,home_pos.longitude, 20);
	waypointPusher(LS_GLOBAL_REL_ALT, TAKE_OFF, false, false, 0, 0, 0, 0, home_pos.latitude,home_pos.longitude, 20);
}

bool mavros_auto::mission_random(void)
{
	//ros_d.waypointPusher(frame,command,isCurrent,autoCont,param1,param2,param3,param4,lat,lon,alt)
	int8_t i = 0;
	static double lat_h = home_pos.latitude, lon_h = home_pos.longitude, alt = 20;
	double lat,lon;
	for (i = 0; i < 12; i++)
	{
#if 0
		int a, b, c, d;
		a = random(2);
		b = random(2);
		c = random(50);
		d = random(50);
		a > 0 ? a = 1 : a = -1;
		b > 0 ? b = 1 : b = -1;
		lat = 0.00002 * c * a+lat_h;
		lon = 0.00002 * d * b+lon_h;
		waypointPusher(LS_GLOBAL_REL_ALT, WAY_POINT, false, false, 0, 0, 0, 0, lat, lon, 20 + i);
#else
		double lon_1m,lat_1m;
		float alt;
		static double count=0;
		lat_1m=0.00000899;
		lon_1m=0.00001141;

		lat = 20*sin(9*count*0.0174533)*lat_1m +lat_h;
		lon = count*lon_1m+lon_h;
		alt=5*sin(36*count*0.0174533)+10;

		waypointPusher(LS_GLOBAL_REL_ALT, WAY_POINT, false, false, 0, 0, 0, 0, lat, lon, alt);
		count+=1;
		ROS_INFO("lat:%f lon:%f count:%f",lat,lon,count);
#endif
	}
	//waypointPusher(LS_GLOBAL_REL_ALT, NAV_LOITER_UNLIM, false, false, 0, 0, 0, 0, lat, lon, 20 + i);
}

