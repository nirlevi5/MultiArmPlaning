/*
 * DualArm.cpp
 *
 *  Created on: Oct 11, 2015
 *      Author: nir
 */

#include "DualArm.h"

DualArm::DualArm(WorldState Initial_state,WorldState Goal_state) {

	initial_state 	= Initial_state;
	goal_state 		= Goal_state;
	plan 			= NULL;

}

DualArm::~DualArm() {
	// TODO Auto-generated destructor stub
}

void DualArm::calc_plan() {

	Node node;
	Node* nodeptr;

	node.state 			= this->initial_state;
	node.action 		= "root ";
	node.parent_index 	= 0;
	node.index 			= 0;
	node.depth 			= 0;


	tree.push_back(node);  				// init tree
	fringe.push_back(&tree.front());   	// init fringe

	// search process
	while (1) {
		// check if there is no solution
		if (fringe.empty()) {
			std::cout<<"empty fringe - no solution"<<std::endl;
			return;
		}
		nodeptr = fringe.back();   // select node from fringe by strategy DFS/BFS/A* than remove it from fringe // LIFO Last In First Out
		fringe.pop_back();		   // remove from the add list

		// Check if a nodeptr was expanded before on this branch
		while ( check_back_tracking(nodeptr) ){
			if (fringe.empty()) {
				std::cout<<"empty fringe - no solution"<<std::endl;
				return;
			}
			nodeptr = fringe.back();   // LIFO Last In First Out
			fringe.pop_back();
		}

		//print this nodeptr
		print_state(nodeptr);

		// Todo - Preform Goal test
		goal_test(nodeptr);


		// Todo - Expand Tree and Fringe
		expand_tree_and_fringe(nodeptr);
		return;
	}
}

bool DualArm::check_back_tracking(Node* nodeptr) {
	// returns true is a node was expanded before and false otherwise
	//	Node* node2check = nodeptr->parent;
	//
	//	while (node2check != NULL) {
	//		if ( false ) // ToDo - actual check operator
	//		{
	//			return true;
	//		}
	//		node2check = node2check->parent;
	//	}
	return false;
}

void DualArm::expand_tree_and_fringe(Node* nodeptr) {

	Node 			newnode,current_node;
	char        	tempstr[100];
	current_node 	= *nodeptr;

	// add nodes by actions
	// -----------------------

	// move_gripper_only(gripper,pos0,pos1)
	// prec: 	empty(gripper)
	//			empty(pos1,gripper)
	//			at(gripper,pos0)
	//			existMP(gripper,pos0,pos1)
	//				foreach block not obstruct MotionPlan
	//				foreach gripper not obstruct MotionPlan
	// effect:	at(gripper,pos1)
	// 			empty(pos0)
	newnode = current_node;
	for(int g = 0 ; g < (int)newnode.state.gripper_list.size() ; g++) {
		for(int p = 0; p < (int)newnode.state.pose_list.size() ; p++) {
			if ( (newnode.state.gripper_list[g].isempty)&&
				 (!newnode.state.pose_list[p].gripperatpose)&&
				 (newnode.state.gripper_list[g].pose_index) != p )
			{
				int gpi = newnode.state.gripper_list[g].pose_index; // gripper pose index
				sprintf(tempstr, "move_gripper_only(%s,%s,%s)",newnode.state.gripper_list[g].gripper_name.c_str(),newnode.state.pose_list[gpi].pose_name.c_str(),newnode.state.pose_list[p].pose_name.c_str());
//				std::cout<<tempstr<<std::endl;

				newnode.action.clear();
				newnode.action.insert(0,tempstr);
				newnode.depth			= newnode.depth + 1;
				newnode.parent_index 	= nodeptr->index;
				newnode.index			= tree.size();

				newnode.state.pose_list[gpi].gripperatpose 	= false;
				newnode.state.gripper_list[g].pose_index 	= p;
				newnode.state.pose_list[p].gripperatpose	= true;

				print_state(&newnode);

				tree.push_back(newnode);
				fringe.push_back(&tree.back());
				newnode = current_node;
			}
		}
	}

	// move_gripper_and_block(gripper,block,pos0,pos1)
	// prec: 	holding(gripper,block)
	//			empty(pos1)
	//			at(gripper,pos0)
	//			existMP(gripper,pos0,pos1)
	//				foreach block not obstruct MotionPlan
	//				foreach gripper not obstruct MotionPlan
	// effect:	at(gripper,pos1)
	//			at(block,pos1)
	// 			empty(pos0)

	for(int g = 0 ; g < (int)newnode.state.gripper_list.size() ; g++) {
		if (newnode.state.gripper_list[g].isempty)	{
			continue;
		}
		for(int p = 0; p < (int)newnode.state.pose_list.size() ; p++) {
			if ( (!newnode.state.pose_list[p].gripperatpose)&&    // no gripper in target position
				 (newnode.state.gripper_list[g].pose_index != p )&&
				 (!newnode.state.pose_list[p].blockatpose) )
			{
				int gpi 	= newnode.state.gripper_list[g].pose_index; // gripper pose index
				int gpbi 	= newnode.state.pose_list[gpi].block_index; // gripper pose block index
//				int pbi 	= newnode.state.pose_list[p].block_index; 	// pose block index

				sprintf(tempstr, "move_gripper_and_block(%s,%s,%s,%s)",newnode.state.gripper_list[g].gripper_name.c_str(),newnode.state.block_list[gpbi].block_name.c_str(),newnode.state.pose_list[gpi].pose_name.c_str(),newnode.state.pose_list[p].pose_name.c_str());

				newnode.action.clear();
				newnode.action.insert(0,tempstr);
				newnode.depth 			= newnode.depth + 1;
				newnode.parent_index 	= nodeptr->index;
				newnode.index			= tree.size();

				newnode.state.pose_list[gpi].gripperatpose 	= false;
				newnode.state.pose_list[gpi].blockatpose 	= false;

				newnode.state.block_list[gpbi].pose_index	= p;
				newnode.state.pose_list[p].block_index 		= gpbi;
				newnode.state.pose_list[gpi].block_index	= -1;

				newnode.state.gripper_list[g].pose_index	= p;

				newnode.state.pose_list[p].gripperatpose 	= true;
				newnode.state.pose_list[p].blockatpose  	= true;

				tree.push_back(newnode);
				fringe.push_back(&tree.back());
				print_state(&newnode);
				newnode = current_node;
			}
		}
	}

	// grasp(gripper,block)
	// prec: 	empty(gripper)
	//          at(gripper.pose,block)
	// effect: 	not empty(gripper)
	for(int g = 0 ; g < (int)newnode.state.gripper_list.size() ; g++) {
		if ( (newnode.state.gripper_list[g].isempty) &&                                          // gripper empty
			 (newnode.state.pose_list[newnode.state.gripper_list[g].pose_index].blockatpose) )	 // block at gripper position
		{
			int gpi 	= newnode.state.gripper_list[g].pose_index; // gripper pose index
			int gpbi 	= newnode.state.pose_list[gpi].block_index; // gripper pose block index

			sprintf(tempstr, "grasp(%s,%s)",newnode.state.gripper_list[g].gripper_name.c_str(),newnode.state.block_list[gpbi].block_name.c_str());

			newnode.action.clear();
			newnode.action.insert(0,tempstr);
			newnode.depth 			= newnode.depth + 1;
			newnode.parent_index 	= nodeptr->index;
			newnode.index			= tree.size();

			newnode.state.gripper_list[g].isempty = false; // gripper is now grasping
			tree.push_back(newnode);
			fringe.push_back(&tree.back());
			print_state(&newnode);
			newnode = current_node;
		}
	}


	// release(gripper)
	// prec: 	not empty(gripper)
	// effect: 	empty(gripper)
	for(int g = 0 ; g < (int)newnode.state.gripper_list.size() ; g++) {
		if (!newnode.state.gripper_list[g].isempty)                                           // gripper not empty
		{
			int gpi 	= newnode.state.gripper_list[g].pose_index; // gripper pose index

			sprintf(tempstr, "release(%s)",newnode.state.gripper_list[g].gripper_name.c_str());

			newnode.action.clear();
			newnode.action.insert(0,tempstr);
			newnode.depth 			= newnode.depth + 1;
			newnode.parent_index 	= nodeptr->index;
			newnode.index			= tree.size();

			newnode.state.gripper_list[g].isempty = true; // gripper is now grasping
			tree.push_back(newnode);
			fringe.push_back(&tree.back());
			print_state(&newnode);
			newnode = current_node;
		}
	}




}




void DualArm::print_state(Node* nodeptr) {

	std::cout<<nodeptr->action<<std::endl;

	std::cout<<"empty positions are:\n   ";
	for(int p = 0; p < (int)nodeptr->state.pose_list.size() ; p++) {
		if (!nodeptr->state.pose_list[p].blockatpose)
			std::cout<< nodeptr->state.pose_list[p].pose_name <<" ";
		else
			std::cout<< "XX ";
	}
	std::cout<<std::endl;

	std::cout<<"populated positions are:\n";
	for(int b = 0; b < (int)nodeptr->state.block_list.size() ; b++) {
		std::cout<<"   block "      << nodeptr->state.block_list[b].block_name <<
				   " is located in: " << nodeptr->state.pose_list[nodeptr->state.block_list[b].pose_index].pose_name << std::endl;
	}

	std::cout<<"grippers position are:\n";
	for(int g = 0 ; g < (int)nodeptr->state.gripper_list.size() ; g++) {

		int pose_index = nodeptr->state.gripper_list[g].pose_index;
		int block_index = nodeptr->state.pose_list[nodeptr->state.gripper_list[g].pose_index].block_index;

		if (nodeptr->state.gripper_list[g].isempty) {
			std::cout<<"   " << nodeptr->state.gripper_list[g].gripper_name <<
			           " is in "<< nodeptr->state.pose_list[pose_index].pose_name <<std::endl;
		}
		else  {
			std::cout<<	"   "        << nodeptr->state.gripper_list[g].gripper_name <<
						" is in "    << nodeptr->state.pose_list[pose_index].pose_name <<
						" grasping " << nodeptr->state.block_list[block_index].block_name<<std::endl;
		}

	}

	std::cout<<"-----------------------------------------------"<<std::endl;
}

bool DualArm::goal_test(Node* nodeptr) {

//	if (nodeptr->state.block_list[b])

	return false;
}
