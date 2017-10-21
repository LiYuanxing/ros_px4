/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <string>
#include <vector>
#include <geometry_msgs/Point.h>

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
geometry_msgs::Point check_next_point(void)
{
	static float target_x = 0, target_y = 0;
	static int i = 0;

	float target_z, next_xz, next_yz;
	bool x_ok = 0, y_ok = 0;
	geometry_msgs::Point next_point;

	if (i % 4 == 0)
	{
		target_x = (i / 4) * 10;
		target_y = 0;
	}
	else if (i % 4 == 1)
	{
		target_x = (i / 4) * 10;
		target_y = 100;
	}
	else if (i % 4 == 2)
	{
		target_x = (i / 4) * 10 + 5;
		target_y = 100;
	}
	else if (i % 4 == 3)
	{
		target_x = (i / 4) * 10 + 5;
		target_y = 0;
	}
#define length 0.5
	if ((target_x - now_pos.x) > length)
		next_xz = now_pos.x + length;
	else if ((target_x - now_pos.x) < -length)
		next_xz = now_pos.x - length;
	else
		x_ok = 1;

	if ((target_y - now_pos.y) > length)
		next_yz = now_pos.y + length;
	else if ((target_y - now_pos.y) < -length)
		next_yz = now_pos.y - length;
	else
		y_ok = 1;

	if (x_ok == 1 && y_ok == 1)
		i++;

	target_z = 10 * sin(next_xz * 0.05) + 10 * cos(next_yz * 0.05) + 25;

	if (fabs(target_z - now_pos.z) < 5)
	{
		next_point.x = target_x;
		next_point.y = target_y;
	}
	else
	{
		next_point.x = now_pos.x;
		next_point.y = now_pos.y;
	}

	//next_point.z = 10 * sin(next_point.x * 0.05) + 10 * cos(next_point.y * 0.05) + 25;
	next_point.z = target_z;
	ROS_INFO("delta:z:%f", next_point.z - now_pos.z);
	ROS_INFO("now x:%.2f y:%.2f next x:%.2f y:%.2f", now_pos.x, now_pos.y, next_point.x, next_point.y);

	return next_point;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "offb_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);

	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

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
	offb_set_mode.request.custom_mode = "guided";

	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;

	if (current_state.mode != "guided" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
	{
		ROS_INFO("Offboard enabled");
	}

	if (!current_state.armed && arming_client.call(arm_cmd) && arm_cmd.response.success)
	{
		ROS_INFO("Vehicle armed");
	}

	ros::Time last_request = ros::Time::now();
	while (ros::ok())
	{
		if (ros::Time::now() - last_request > ros::Duration(0.3))
		{
			last_request = ros::Time::now();

			//ROS_INFO("Send Point");local_pos

			pose.pose.position = check_next_point();
			local_pos_pub.publish(pose);
	}
	ros::spinOnce();
	rate.sleep();
}

return 0;
}
