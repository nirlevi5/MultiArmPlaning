//============================================================================
// Name        : DualArmTaskPlaning.cpp
// Author      : Nir
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "DualArm.h"


int main() {

	PoseState  		p1,p2,p3,p4,p5,p6;
	GripperState 	g1,g2;
	BlockState 		b1,b2;
	WorldState 		initial_state,goal_state;

	// initial state definition
	p1.pose_name 			= "p1";
	p1.blockatpose			= false;
	p1.gripperatpose		= false;
	p1.pose.pose[0]			=  0.5; // x
	p1.pose.pose[1]			= -0.2; // y
	p1.pose.pose[2]			=  0;  	// theta
	p1.block_index			=  -1;

	initial_state.pose_list.push_back(p1);

	p2.pose_name 			= "p2";
	p2.blockatpose			= false;
	p2.gripperatpose		= false;
	p2.pose.pose[0]			=  0.5; // x
	p2.pose.pose[1]			=  0.2; // y
	p2.pose.pose[2]			=  0;  	// theta
	p2.block_index			=  -1;

	initial_state.pose_list.push_back(p2);

	p3.pose_name 			= "p3";
	p3.blockatpose			= false;
	p3.gripperatpose		= false;
	p3.pose.pose[0]			= -0.5; // x
	p3.pose.pose[1]			= -0.2; // y
	p3.pose.pose[2]			= -M_PI;// theta
	p3.block_index			=  -1;

	initial_state.pose_list.push_back(p3);

	p4.pose_name 			= "p4";
	p4.blockatpose			= false;
	p4.gripperatpose		= false;
	p4.pose.pose[0]			= -0.5; // x
	p4.pose.pose[1]			=  0.2; // y
	p4.pose.pose[2]			= -M_PI;// theta
	p4.block_index			=  -1;

	initial_state.pose_list.push_back(p4);

	p5.pose_name 			= "p5";
	p5.blockatpose			= false;
	p5.gripperatpose		= false;
	p5.pose.pose[0]			=  0;   // x
	p5.pose.pose[1]			= -0.4; // y
	p5.pose.pose[2]			= -M_PI_2;// theta
	p5.block_index			=  -1;

	initial_state.pose_list.push_back(p5);

	p6.pose_name 			= "p6";
	p6.blockatpose			= false;
	p6.gripperatpose		= false;
	p6.pose.pose[0]			=  0;   // x
	p6.pose.pose[1]			=  0.4; // y
	p6.pose.pose[2]			=  M_PI_2;// theta
	p6.block_index			=  -1;

	initial_state.pose_list.push_back(p6);

	// define block states
	b1.block_name 			= "b1";
	b1.pose_index  			= 1;      //real pose - 1
	initial_state.pose_list[b1.pose_index].blockatpose = true;
	initial_state.pose_list[b1.pose_index].block_index = 0;    // real block - 1

	initial_state.block_list.push_back(b1);

	g1.gripper_name 		= "g1";
	g1.isempty 				= false;
	g1.pose_index			= 1;    // real pose - 1
	initial_state.pose_list[g1.pose_index].gripperatpose = true;
	g1.joint_angles[0] 		= 0;
	g1.joint_angles[1] 		= 0;
	g1.joint_angles[2] 		= 0;

	initial_state.gripper_list.push_back(g1);

	// goal state definition
	b1.pose_index = 3;

	goal_state.block_list.push_back(b1);

	DualArm dualarm(initial_state,goal_state);
	dualarm.calc_plan();
	return 0;
}
