/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */
#include "mavros_auto.h"

int main(int argc, char **argv)
{

	mavros_auto ros_d;
	ros::Rate rate(20.0);

	while (ros::ok())
	{
		if(ros_d.now_global_pos.latitude!=0||ros_d.now_global_pos.longitude!=0)
			break;
		ros::spinOnce();
		rate.sleep();
	}
	//ros_d.waypointPusher(pusher,client,node,frame,command,isCurrent,autoCont,param1,param2,param3,param4,lat,lon,alt)
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh,     0, 16, false, false, 0, 0, 0, 0, 1, 1, 584.070007324); //home
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh, FRAME, 22, false, false, 0, 0, 0, 0, 2, 2, 20); //take off
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh, FRAME, 16, false, false, 0, 0, 0, 0, 3, 3, 20); //way point
	ros_d.waypointPusher(ros_d.wayPusher, ros_d.pushClient, ros_d.nh, FRAME, 21, false, false, 0, 0, 0, 0, 4, 4, 20); //land


	// wait for FCU connection
	while (ros::ok() && ros_d.current_state.connected)
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

	if (!ros_d.current_state.armed && ros_d.arming_client.call(arm_cmd) && arm_cmd.response.success)
	{
		ros::spinOnce();
		rate.sleep();
		ROS_INFO("auto armed");

		mavros_msgs::CommandTOL tol;
		tol.request.altitude = 20;
		tol.request.yaw = 50;
		tol.request.min_pitch = 10;
		offb_set_mode.request.custom_mode = "guided";

		if (ros_d.current_state.mode != "guided" && ros_d.set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
		{
			ROS_INFO("guided enabled");
			ros_d.set_mode_client.call(offb_set_mode);
			ros::spinOnce();
			rate.sleep();
		}
		//ROS_INFO("take off:%d", takeoff_client.call(tol));
	}

//	if (ros_d.current_state.mode != "auto" && ros_d.set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
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
