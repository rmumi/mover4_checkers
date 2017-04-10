#ifndef moving_H
#define moving_H

// #include <termios.h>
// #include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <list>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>


struct robotState
{
	float p[6];		// cart position
	float j[6];		// joint position
	int errorCode[6];
	float duration;	// duration for motion; needed for actionServer
};



#endif
