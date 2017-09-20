/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include "mavros_auto.h"


mavros_auto  ros_d;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "auto_node");
	ros::Rate rate(20.0);

	while (ros::ok()&& !ros_d.preparation())
	{
		ros::spinOnce();
		rate.sleep();
	}
	//ros_d.waypointPusher(pusher,client,node,frame,command,isCurrent,autoCont,param1,param2,param3,param4,lat,lon,alt)
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh,     0, 16, false, false, 0, 0, 0, 0, 1, 1, 584.070007324); //home
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh, FRAME, 22, false, false, 0, 0, 0, 0, 2, 2, 20); //take off
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh, FRAME, 16, false, false, 0, 0, 0, 0, 3, 3, 20); //way point
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh, FRAME, 21, false, false, 0, 0, 0, 0, 4, 4, 20); //land

	ros_d.arm_copter();

	ros::Time last_request = ros::Time::now();

	while (ros::ok())
	{
		if (ros::Time::now() - last_request > ros::Duration(0.3))
		{
			last_request = ros::Time::now();
		}
		ros::spinOnce();
		rate.sleep();
	}
	//local_pos_pub.publish(pose);
	return 0;
}
