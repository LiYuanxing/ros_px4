/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include <string>
#include <vector>
#include <ros/ros.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointPush.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#define FRAME 3

mavros_msgs::State current_state;
geometry_msgs::Point now_pos;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped msg)
{
	now_pos.x = msg.pose.position.x;
	now_pos.y = msg.pose.position.y;
	now_pos.z = msg.pose.position.z;
}

bool waypointPusher(mavros_msgs::WaypointPush &pusher, ros::ServiceClient client, ros::NodeHandle node, int frame, int command,
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);

	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	ros::ServiceClient pushClient;
	mavros_msgs::WaypointPush wayPusher;

//	waypointPusher(pusher,client,node,frame,command,isCurrent,autoCont,param1,param2,param3,param4,lat,lon,alt)

	waypointPusher(wayPusher, pushClient, nh, FRAME, 16, true, true, 0, 0, 0, 0, -35.3641624451, 149.166412354, 20);
	waypointPusher(wayPusher, pushClient, nh, FRAME, 16, true, true, 0, 0, 0, 0, -35.3641634451, 149.166412354, 20);
	waypointPusher(wayPusher, pushClient, nh, FRAME, 16, true, true, 0, 0, 0, 0, -35.3641644451, 149.166412354, 20);


	//the setpoint publishing rate MUST be faster than 2Hz
	ros::Rate rate(20.0);

	// wait for FCU connection
	while (ros::ok() && current_state.connected)
	{
		ros::spinOnce();
		rate.sleep();
	}

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2;

	mavros_msgs::SetMode offb_set_mode;

	offb_set_mode.request.custom_mode = "auto";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	if (!current_state.armed && arming_client.call(arm_cmd) && arm_cmd.response.success)
	{
		ros::spinOnce();
		rate.sleep();
		ROS_INFO("auto armed");

		mavros_msgs::CommandTOL tol;
		tol.request.altitude = 20;
		tol.request.yaw = 50;
		tol.request.min_pitch = 10;
		offb_set_mode.request.custom_mode = "guided";

		if (current_state.mode != "guided" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
		{
			ROS_INFO("guided enabled");
			set_mode_client.call(offb_set_mode);
			ros::spinOnce();
			rate.sleep();
		}
		//ROS_INFO("take off:%d", takeoff_client.call(tol));
	}

//	if (current_state.mode != "auto" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
//	{
//		ROS_INFO("auto enabled");
//	}

	ros::Time last_request = ros::Time::now();
	while (ros::ok())
	{
		if (ros::Time::now() - last_request > ros::Duration(0.3))
		{
			last_request = ros::Time::now();

			//ROS_INFO("Send Point");

			pose.pose.position.x++;
			pose.pose.position.y++;
			pose.pose.position.z++;

			//local_pos_pub.publish(pose);
		}
		ros::spinOnce();
		rate.sleep();
	}
	//local_pos_pub.publish(pose);
	return 0;
}
