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
	ros::Rate rate(20.0);

	//wait connect and gps
	while (ros::ok() && !ros_d.preparation())
	{
		ros::spinOnce();
		rate.sleep();
	}

	//ros_d.waypointPusher(frame,command,isCurrent,autoCont,param1,param2,param3,param4,lat,lon,alt)

	ros_d.waypointPusher(LS_GLOBAL_REL_ALT, TAKE_OFF, false, false, 0, 0, 0, 0, ros_d.home_pos.latitude, ros_d.home_pos.longitude, 20);
	ros_d.waypointPusher(LS_GLOBAL_REL_ALT, TAKE_OFF, false, false, 0, 0, 0, 0, ros_d.home_pos.latitude, ros_d.home_pos.longitude, 20);

	double lat = ros_d.home_pos.latitude, lon = ros_d.home_pos.longitude, alt = 20;
	for (int8_t i = 0; i < 20; i++)
	{
		int j,k;
		random(3)>1?j=1:j=-1;
		random(3)>1?k=1:k=-1;
		lat += 0.00002 * random(60)*j;
		lon += 0.00002 * random(60)*k;
		ros_d.waypointPusher(LS_GLOBAL_REL_ALT, WAY_POINT, false, false, 0, 0, 0, 0, lat, lon, 20 + i);
	}
	ros_d.waypointPusher(LS_GLOBAL_REL_ALT, RTL, false, false, 0, 0, 0, 0, ros_d.home_pos.latitude, ros_d.home_pos.longitude, 20);

	ros_d.arm_copter();
	ros_d.set_auto();

	ros::Time last_request = ros::Time::now();

	while (ros::ok())
	{
		if (ros::Time::now() - last_request > ros::Duration(1))
		{
			last_request = ros::Time::now();
		}
		ros::spinOnce();
		rate.sleep();
	}
	//local_pos_pub.publish(pose);
	return 0;
}
