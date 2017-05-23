#ifndef MOVING_H
#define MOVING_H

// #include <termios.h>
// #include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <list>
#include <vector>
#include <deque>
#include <algorithm>
#include "matrix.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

// #include <actionlib/server/simple_action_server.h>

// #include <control_msgs/FollowJointTrajectoryAction.h>
// #include <control_msgs/GripperCommandAction.h>

using namespace ch;

struct robotState {
	double p[6];		// cart position
	double j[4];		// joint position
	// int errorCode[6];
	// double duration;	// duration for motion; needed for actionServer
};

const static int update_f = 20;

const static double PI = 3.14159265359;
const static double deg2rad = PI / 180.0;
const static double rad2deg = 180.0 / PI;
const static double EPS = 1e-6;

// kinematic length of the Mover4 robot arm in mm
const static double lz0 = 130.0;	// base
const static double lz1 = 62.5;		// first joint
const static double lz2 = 190.0;	// upper arm
const static double lz3 = 220.0;	// lower arm
const static double lz4 = 48.0;		// hand

const static double EpsilonCenter = 5.0;
#define DEFAULT
#ifdef DEFAULT
const static double a0 = 220;//206;
const static double a1 = 190;//190
const static double a2 = 220;//225
const static double a3 = 48 + 95;  // TODO a bit skewed, but OK for now // skewed for 12 mm to the left <- 15 to the back ^
#else
const static double a0 = 220;//206;
const static double a1 = 200;//190
const static double a2 = 230;//225
const static double a3 = 150;  // TODO a bit skewed, but OK for now // skewed for 12 mm to the left <- 15 to the back ^
#endif

static Matrix DH_par ({  // theta d alpha a
{0.0, a0, -PI/2, 0},
{-PI/2, 0, 0, a1},
{0.0, 0, 0, a2},
{0.0, 0, 0, a3},
});

robotState InvKine(const robotState&, int);

robotState ForKine(const robotState&);

using std::vector;
using std::deque;

namespace ch {
class RobotAction {
public:
    RobotAction(int flags, const robotState &q_joints, int orient=0, double wait=0.) {
        _q = q_joints;
        gripper_open = flags&1;
        gripper_close = flags&2;
        to_point = flags&4;
        mid_point = flags&8;
        waiting = flags&16;
        this->wait = wait*update_f;
    }
    bool gripper_open, gripper_close, to_point, mid_point, waiting;
    int wait;
    robotState _q;
};
}

#endif //MOVING_H
