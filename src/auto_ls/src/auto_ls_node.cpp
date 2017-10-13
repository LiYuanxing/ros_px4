/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include "mavros_auto.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_node");
	mavros_auto ros_d;
	int test = 0;
	int current;
	ros::Rate rate(100.0);

	//wait connect and gps
	while (ros::ok() && !ros_d.preparation())
	{
		ros::spinOnce();
		rate.sleep();
	}
	ros_d.mission_home_takeoff();
	ros_d.mission_random();
	ros_d.mission_set_current(0);

	ros_d.arm_copter();
	ros_d.set_auto();

	ros::Time last_request1 = ros::Time::now();
	ros::Time last_request2 = ros::Time::now();

	while (ros::ok())
	{
		if ((ros_d.way.max-1)==ros_d.way.current)
		{
			if (ros_d.way.max > 700)
			{
				ros_d.mission_clear();
				ros_d.mission_home_takeoff();
				ros_d.mission_random();
				ros_d.mission_set_current(2);
				test++;
				ROS_INFO("new buf test:%d ", test);
			}
			else
			{
				current = ros_d.way.current;
				ros_d.mission_random();
				ros_d.mission_set_current(current);
				test++;
				ROS_INFO("old buf test:%d ", test);
			}
		}

		if (ros::Time::now() - last_request1 > ros::Duration(0.05)) //20Hz
		{
			last_request1 = ros::Time::now();
		}
		if (ros::Time::now() - last_request2 > ros::Duration(0.05)) //20Hz
		{
			last_request2 = ros::Time::now();
		}
		ros::spinOnce();
		rate.sleep();
	}
	//local_pos_pub.publish(pose);
	return 0;
}
