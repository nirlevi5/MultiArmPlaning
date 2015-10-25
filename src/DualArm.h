/*
 * DualArm.h
 *
 *  Created on: Oct 11, 2015
 *      Author: nir
 */
#include <iostream>
#include <list>
#include <vector>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <iterator>

#ifndef DUALARM_H_
#define DUALARM_H_
//typedef Pose , PoseState , BlockState , GripperState;

struct Pose {
	double pose[3];
};

struct PoseState {
	std::string 	pose_name;
	bool 			blockatpose;
	bool 			gripperatpose;
	Pose			pose;
	int	block_index;
};

struct BlockState {     // all the occupied positions by blocks
	std::string 	block_name;
	int	pose_index;
};

struct GripperState {
	std::string 	gripper_name;
	bool    		isempty;
	int	pose_index;
	double			joint_angles[3];
};

struct WorldState {
	std::vector<BlockState> 	block_list;
	std::vector<GripperState> 	gripper_list;
	std::vector<PoseState> 		pose_list;
};

struct Node {
	WorldState	    state;
	std::string 	action;  //the action that result with this state
	int   	index;
	int   	parent_index;
	int 			depth;
};

class DualArm {
public:
	DualArm(WorldState Initial_state,WorldState Goal_state);
	virtual ~DualArm();
	void calc_plan();
	void expand_tree_and_fringe(Node* nodeptr);
	bool goal_test(Node* nodeptr);
	bool check_back_tracking(Node* nodeptr);
	void print_state(Node* nodeptr);

	WorldState 	initial_state;
	WorldState 	goal_state;
	Node* 		plan;

	std::vector<Node> tree;			// stores all the nodes in the search tree
	std::vector<Node*> fringe;		// all the nodes in the search tree that we not yet explored


};

#endif /* DUALARM_H_ */
