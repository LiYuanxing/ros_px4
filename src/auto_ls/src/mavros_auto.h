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
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include "stdlib.h"

#define LS_GLOBAL 			 0u
#define LS_LOCAL_NED 		 1u
#define LS_MISSION 		 	 2u
#define LS_GLOBAL_REL_ALT 	 3u
#define LS_LOCAL_ENU 		 4u


#define WAY_POINT			16
#define RTL					20
#define LAND				21
#define TAKE_OFF			22

#define random(x) (rand()%x)


//enum { CMD_DO_SET_MODE = 176u };
//enum { CMD_DO_JUMP = 177u };
//enum { CMD_DO_CHANGE_SPEED = 178u };
//enum { CMD_DO_SET_HOME = 179u };
//enum { CMD_DO_SET_RELAY = 181u };
//enum { CMD_DO_REPEAT_RELAY = 182u };
//enum { CMD_DO_SET_SERVO = 183u };
//enum { CMD_DO_REPEAT_SERVO = 184u };
//enum { CMD_DO_CONTROL_VIDEO = 200u };
//enum { CMD_DO_SET_ROI = 201u };
//enum { CMD_DO_MOUNT_CONTROL = 205u };
//enum { CMD_DO_SET_CAM_TRIGG_DIST = 206u };
//enum { CMD_DO_FENCE_ENABLE = 207u };
//enum { CMD_DO_PARACHUTE = 208u };
//enum { CMD_DO_INVERTED_FLIGHT = 210u };
//enum { CMD_DO_MOUNT_CONTROL_QUAT = 220u };
//enum { CMD_PREFLIGHT_CALIBRATION = 241u };
//enum { CMD_MISSION_START = 300u };
//enum { CMD_COMPONENT_ARM_DISARM = 400u };
//enum { CMD_GET_HOME_POSITION = 410u };
//enum { CMD_START_RX_PAIR = 500u };
//enum { CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520u };
//enum { CMD_DO_TRIGGER_CONTROL = 2003u };
//enum { NAV_WAYPOINT = 16u };
//enum { NAV_LOITER_UNLIM = 17u };
//enum { NAV_LOITER_TURNS = 18u };
//enum { NAV_LOITER_TIME = 19u };
//enum { NAV_RETURN_TO_LAUNCH = 20u };
//enum { NAV_LAND = 21u };
//enum { NAV_TAKEOFF = 22u };


class mavros_auto
{
public:
	mavros_auto();
	~mavros_auto();

	struct lat_lon_alt_t
	{
		double lat;
		double lon;
		double alt;
	};
	struct waypoint_t
	{
		int current;
		int max;
	};

	ros::NodeHandle nh;
	ros::ServiceClient pushClient;

	mavros_msgs::State current_state;
	geometry_msgs::Point now_pos;
	mavros_msgs::GlobalPositionTarget now_global_pos;
	mavros_msgs::HomePosition home_pos;

	mavros_msgs::WaypointPush wayPusher;
	mavros_msgs::WaypointList curPoint;
	waypoint_t	way;

	ros::Subscriber state_sub;
	ros::Subscriber position_sub;
	ros::Subscriber cur_point_pub;
	ros::Subscriber global_position_sub;
	ros::Subscriber home_point_sub;

	ros::Publisher local_pos_pub;

	ros::ServiceClient arming_client;
	ros::ServiceClient takeoff_client;
	ros::ServiceClient set_mode_client;


	void state_cb(const mavros_msgs::State::ConstPtr& msg);
	void pose_cb(const geometry_msgs::PoseStamped msg);
	void global_cb(const mavros_msgs::GlobalPositionTarget msg);
	void home_cb(const mavros_msgs::HomePosition msg);
	void cur_point_cb(const mavros_msgs::WaypointList msg);

	lat_lon_alt_t cal_pos(double x,double y,double z);
	bool waypointPusher(int frame, int command, bool isCurrent, bool autoCont, float param1, float param2, float param3, float param4,
			float lat, float lon, float alt);
	bool preparation();
	bool arm_copter();
	bool set_auto();

};

#endif /* MAVROS_AUTO_H_ */
