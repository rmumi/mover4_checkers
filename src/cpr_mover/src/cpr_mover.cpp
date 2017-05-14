/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Commonplace Robotics GmbH
 *  http://www.commonplacerobotics.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name Commonplace Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// Created on: 	October 26th, 2014
// V01.4 October 7th, 2016:	JointMinMax are set differently for M4 and M6, increased velocity
// V01.5 Nov 11th, 2016:	Different definition of Joint topic wih 6+2 resp. 4+2 values

/*
	Functionality:
	* Establishes a main loop to control the robot arm Mover4 or Mover6
	* Connects to a Mover robot using the PCAN adapter
	* Allows to move the joints with the cpr_rviz_plugin
	* Forwards joints and gripper joints in a joint_states message
	* Accepts joint_trajectory_actions from e.g. moveit
	* Accepts gripperCommandActions from e.g. moveit
	* Publishes the current joint positions and the robot status

	ToDo:
	* Include the CPRRS232 USB-CAN-adapter
	* improve structure and quality, debug
*/


#include <cpr_mover.h>
#include <list>

using namespace std;

const static double PI = 3.14159265359;

double gripperJointStatus = 0.0;			// State of the gripper joints. This is a workaround, the real joints are commanded via digital io
double gripperJointMax = 35.0 * PI / 180.0;	// Opening width

double deg2rad = PI / 180.0;
double rad2deg = 180.0 / PI;

//****************************************************
void quit(int sig)
{
  ros::shutdown();
  exit(0);
}

//******************** MAIN ************************************************
int main(int argc, char** argv)
{
	ros::init(argc, argv, "cpr_mover_robot");
	ros::NodeHandle n2;

	// Start the robot
	cpr_robots::cpr_mover robot;
	robot.init();
	robot.mainLoop();		//spinning is done inside the main loop

  	signal(SIGINT,quit);
	return(0);
}


namespace cpr_robots{


//*************************************************************************************
cpr_mover::cpr_mover(){

	ROS_INFO("CPR-Mover Mainloop V01.5 Nov. 11th, 2016");
	// which robot to operate?
	// should be defined in the launch file / parameter server
	//<param name="robot_type" value="mover4"/>
	std::string global_name, relative_name, default_param;
	if (n.getParam("/robot_type", global_name)){
		ROS_INFO("%s", global_name.c_str());
	}else{
		ROS_INFO("no robot name found");
	}
}


//*************************************************************************************
cpr_mover::~cpr_mover(){

}

//*************************************************************************************
void cpr_mover::init(){
	ROS_INFO("...initing...");

	nrOfJoints = 4;
	kin.nrOfJoints = 4;
	kin.SetJointMinMax(0);
	itf.Init("mover4");

	msgJointsCurrent.header.stamp = ros::Time::now();
	msgJointsCurrent.name.resize(6);
	msgJointsCurrent.position.resize(6);
	msgJointsCurrent.name[0] ="Joint0";
	msgJointsCurrent.position[0] = 0.0;
	msgJointsCurrent.name[1] ="Joint1";
	msgJointsCurrent.position[1] = 0.0;
	msgJointsCurrent.name[2] ="Joint2";
	msgJointsCurrent.position[2] = 0.0;
	msgJointsCurrent.name[3] ="Joint3";
	msgJointsCurrent.position[3] = 0.0;
	msgJointsCurrent.name[4] ="Gripper1";		// Joints 5 and 6 are gripper joints
	msgJointsCurrent.position[4] = 0.0;
	msgJointsCurrent.name[5] ="Gripper2";
	msgJointsCurrent.position[5] = 0.0;


	setPointState.j[0] =   0.0;			// values are initialized with 6 field to be usable for Mover4 and Mover6
	setPointState.j[1] = -20.0;
	setPointState.j[2] =  20.0;
	setPointState.j[3] =  20.0;
	setPointState.j[4] =   0.0;			// This was 30
	setPointState.j[5] =   0.0;

	jointMaxVelocity[0] = 40.0;
	jointMaxVelocity[1] = 40.0;
	jointMaxVelocity[2] = 40.0;
	jointMaxVelocity[3] = 40.0;
	jointMaxVelocity[4] = 40.0;
	jointMaxVelocity[5] = 40.0;

	// when starting up (or when reading the HW joint values) the target position has to be aligned with the setPoint position
	for(int i=0; i<nrOfJoints; i++)
		targetState.j[i] = setPointState.j[i];

	ovrPercent = 50.0;
	cycleTime = 50.0;

	// Publish the current joint states
	pubJoints = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

	subJointPos = n.subscribe<sensor_msgs::JointState>("/CPRMoverJointPos", 1, &cpr_mover::jointPosCallback, this);

	msgErrorStates.data = "error 0x04";
	pubErrorStates = n.advertise<std_msgs::String>("/CPRMoverErrorCodes", 1);

	subCommands = n.subscribe<std_msgs::String>("/CPRMoverCommands", 1, &cpr_mover::commandsCallback, this);
}


//****************************************************************
void cpr_mover::mainLoop()
{
		ROS_INFO("Starting Mover Main Loop");

	 	for(;;)
		{
		// MotionGeneration();			// Generate the joint motion and actuate the gripper
		CommunicationHW();			// Forward the new setpoints to the hardware
		CommunicationROS();			// Publish the joint states and error info
		// Move();
		if(!itf.GetConnectionStatus())
			for(int i = 0; i < 4; i++) currentState.j[i] = setPointState.j[i];

		ros::spinOnce();
		ros::Duration(cycleTime/1000.0).sleep();		// main loop with 20 Hz.

		}
	ROS_INFO("Closing Mover Main Loop");

} //endof mainLoop


//*************************************************************************************
// Here we receive the discrete commands like Connect, Reset, Enable
// the commands are forwarded to the interface class
void cpr_mover::commandsCallback(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("CMD: %s ", msg->data.c_str());  // This was commented
	std::string rec = msg->data;

	if( rec == "Connect"){
		itf.Connect();
		ROS_INFO("Connect");
	}
	else if( rec == "Reset"){
		itf.GetJoints( setPointState.j );
		for(int i=0; i<nrOfJoints; i++)
			targetState.j[i] = setPointState.j[i];
		itf.ResetError();
		ROS_INFO("Reset and load joint position");
	}
	else if( rec == "Enable" ){
		itf.EnableMotors();
		ROS_INFO("Enable");
	}
	else if( rec == "GripperOpen" ){
		itf.SetIO(3, 1, true);
		itf.SetIO(3, 0, true);
		gripperJointStatus = gripperJointMax;		// Workaround: the gripper should be in this position now, even if it is commanded by digital IO. This value is send to RViz
		ROS_INFO("GripperOpen");
	}
	else if( rec == "GripperClose" ){
		itf.SetIO(3, 1, true);
		itf.SetIO(3, 0, false);
		gripperJointStatus = 0.0;
		ROS_INFO("GripperClose");
	}
	else if( rec[0] == 'O' && rec[1] == 'v' && rec[2] == 'e'){
		int l = rec.length();
		std::string ovr = rec.substr(9, l-1);
		int newovr = atoi(ovr.c_str());

		if(newovr > 100) newovr = 100;
		if(newovr < 0) newovr = 0;
		ovrPercent = newovr;

		ROS_INFO("New Override %d", newovr);
	}
}

//*****************************************************************
void cpr_mover::jointPosCallback(const sensor_msgs::JointState::ConstPtr& msg){
	double tmp = 0.0;
	int i=0;
	for(i=0; i<nrOfJoints; i++){
		tmp = msg->position[i];
		setPointState.j[i] = tmp * rad2deg;
	}
}

//***************************************************************
// Forward the new setpoints to the hardware
void cpr_mover::CommunicationHW(){

	if(!itf.GetConnectionStatus())		// if HW is not connected then skip the rest
		return;

	static bool first = true;
	float ju[6];

	itf.GetJoints( ju );
	for(int i = 0; i < 4; i++) currentState.j[i] = ju[i];
	if(first) for(int i = 0; i < 4; i++) setPointState.j[0] = currentState.j[0];

	first = false;

	for(int i = 0; i < 4; i++) if(setPointState.j[i] != setPointState.j[i]) {printf("NaN");return;}
	if(kin.CheckJointMinMax( setPointState.j )) {printf("Over");return;}

	for(int i=0; i<nrOfJoints; i++) {
		itf.SetJoints( setPointState.j );
	}
    // static count_br = 0;
	// if(count_br++ % 32 == 0) {
	// 	float ju[6];
	// 	itf.GetJoints( ju );
	// 	for(int i = 0; i < 4; i++) setPointState.j[i] = ju[i];
	// 	//itf.GetPID();
	// }

	// for(int i=0; i<4;i++) setPointState.j[i] = ju[i];

}


//***************************************************************
// forward the current joints to RViz etc
void cpr_mover::CommunicationROS(){

	msgJointsCurrent.header.stamp = ros::Time::now();
	msgJointsCurrent.position[0] = deg2rad * currentState.j[0];		// Robot CAN communication works in degree
	msgJointsCurrent.position[1] = deg2rad * currentState.j[1];
	msgJointsCurrent.position[2] = deg2rad * currentState.j[2];
	msgJointsCurrent.position[3] = deg2rad * currentState.j[3];

	msgJointsCurrent.position[4] = gripperJointStatus;					// The two gripper joints. Workaround, in the Mover robots these are digital IO controlled
	msgJointsCurrent.position[5] = gripperJointStatus;
	pubJoints.publish(msgJointsCurrent);								// ROS communication works in Radian

	msgErrorStates.data = itf.GetErrorMsg();

	pubErrorStates.publish(msgErrorStates);
}

}
