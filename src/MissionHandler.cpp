#include "MissionHandler.h"

MissionHandler::MissionHandler(std::string opt_traj_ref_file)
{

	//Init optimal trajectory that lies in the file opt_traj_ref_file (i.e ref.txt).
	OptTrajectory = new OptimalTrajectory(opt_traj_ref_file);

	numberOfMissions = 1;
	referenceWaypoints.resize(numberOfMissions);
	nextReferenceWaypoints.resize(numberOfMissions);

	/****************************************************************/
	/*** Setup scenario 2 *******************************************/
	/****************************************************************/
	/*   referenceWaypoints[1].push_back(WayPoint(123.0, 50.0, 6,  M_PI/2, 10, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(123.0, 85.0, 6,  M_PI/2, 11, Behaviour::forwardOnRoad,Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(70.0, 93.0, 8, -M_PI, 12, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(45.0, 92.0, 3, -165*M_PI/180, 13, Behaviour::forwardOnRoad, Behaviour::Uturn, 20));
	referenceWaypoints[1].push_back(WayPoint(95.0, 97.0, 3, 14, Behaviour::Uturn, Behaviour::forwardOnRoad, 1));
	referenceWaypoints[1].push_back(WayPoint(110.0, 97.0, 5, 0, 15, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));

	referenceWaypoints[1].push_back(WayPoint(127, 70.0, 10, -M_PI/2, 16, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(127, 60.0, 15, -M_PI/2, 17, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(127, 20.0, 25, -M_PI/2, 18, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(127, -20.0, 25, -M_PI/2, 19, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(127, -60.0, 15, -M_PI/2, 20, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[1].push_back(WayPoint(127, -80.0, 10, -M_PI/2, 21, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));

	for(int i = 1; i<3; i++)
	{
	referenceWaypoints[1].push_back(WayPoint(127, -80 - 100*i, 10, -M_PI/2, 21 + i, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	}

	/****************************************************************/
	/*** Setup scenario 3 *******************************************/
	/****************************************************************/
	/*    referenceWaypoints[2].push_back(WayPoint(127, 70.0, 10, -M_PI/2, 16, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[2].push_back(WayPoint(127, 60.0, 15, -M_PI/2, 17, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[2].push_back(WayPoint(127, 20.0, 25, -M_PI/2, 18, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[2].push_back(WayPoint(127, -20.0, 25, -M_PI/2, 19, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[2].push_back(WayPoint(127, -60.0, 25, -M_PI/2, 20, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	referenceWaypoints[2].push_back(WayPoint(127, -80.0, 25, -M_PI/2, 21, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));

	for(int i = 1; i<3; i++)
	{
	referenceWaypoints[2].push_back(WayPoint(127, -80 - 100*i, 25, -M_PI/2, 21 + i, Behaviour::forwardOnRoad, Behaviour::forwardOnRoad, 20));
	}
	*/
}


void  MissionHandler::set_mission_plan(bool safe_mode, State egoState, RectObstacle rectObstacle)
{

	if(safe_mode)
	{
		cout << "Generate mission: (SafetyMode)" << endl << endl;
		cout << "state_of_car: " << "xpos: " << egoState.x << " ypos: " << egoState.y << endl;

		//Set starting Checkpoint as egostate.
		//referenceWaypoints[0].push_back(WayPoint(egoState.x, egoState.y, egoState.v, egoState.heading, 0, Safety_Behaviour, Safety_Behaviour, 10));

		//Find nearest point on opt.traj. wrt distance from center of egostate(the car).
		int index_ego_state = OptTrajectory->find_index(egoState.x,egoState.y);


		cout << "index_ego_state: " << index_ego_state << endl;
		cout << "state_near_ego_state: " << "xpos: " << OptTrajectory->get_optimal_trajectory_state(index_ego_state).x << " ypos: " << OptTrajectory->get_optimal_trajectory_state(index_ego_state).y << endl;

		//Find nearest point on opt.traj. wrt distance from center of the obstacle.
		int index_rect_obstacle_state = OptTrajectory->find_index(rectObstacle.pos.x,rectObstacle.pos.y);
		cout << endl << "state_of_obstacle: " << "xpos: " << rectObstacle.pos.x << " ypos: " << rectObstacle.pos.y << endl;

		cout << "state_near_obstacle_state: " << "xpos: " << egoState.x << " ypos: " << egoState.y << endl;

		cout << "index_obst_state: " << index_rect_obstacle_state << endl;
		cout << "state_near_obst_state: " << "xpos: " << OptTrajectory->get_optimal_trajectory_state(index_rect_obstacle_state).x << " ypos: " << OptTrajectory->get_optimal_trajectory_state(index_rect_obstacle_state).y << endl;

		//Calc distance from index_ego_state to index_rect_obstacle_state
		float length_to_middle_of_obstacle = OptTrajectory->integration_from_to(index_ego_state, index_rect_obstacle_state);

		if(length_to_middle_of_obstacle > 50.0)
			length_to_middle_of_obstacle=50.0;
		else if(length_to_middle_of_obstacle<30.0)
			length_to_middle_of_obstacle=30.0;

		cout << endl << "Length from car to obstacle intersection: " << length_to_middle_of_obstacle << endl;

		//Add length_to_middle_of_obstacle from state_near_obst_state along optimal trajectory as good way-point position.
		int index_final_checkpoint = OptTrajectory->get_index_from_state_add_length(index_rect_obstacle_state, length_to_middle_of_obstacle);
		State goal_state = OptTrajectory->get_optimal_trajectory_state(index_final_checkpoint);
		cout << "index_final_goal_mission: " << index_final_checkpoint << endl;

		cout << "state_of_goal_mission: " << "xpos: " << goal_state.x << " ypos: " << goal_state.y << "angle: "<< goal_state.heading << endl;

		//Push back calc state.
		referenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, 1,Safety_Behaviour, Safety_Behaviour, 10));
		//nextReferenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v,goal_state.heading,1,Safety_Behaviour, Safety_Behaviour, 10));

		int index_Add_final_checkpoint = OptTrajectory->get_index_from_state_add_length(index_final_checkpoint, length_to_middle_of_obstacle);
		goal_state = OptTrajectory->get_optimal_trajectory_state(index_Add_final_checkpoint);

		referenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading,2,Safety_Behaviour, Safety_Behaviour, 10));
		nextReferenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y,egoState.v,goal_state.heading,2,Safety_Behaviour, Safety_Behaviour, 10));


		/*int i = index_Add_final_checkpoint + 20;
		int next_id = 3;
		for (i; i<(OptTrajectory->get_optimal_vector_size() - 35); i += 35)
		{
			int index_Add_final_checkpoint = OptTrajectory->get_index_from_state_add_length(i, length_to_middle_of_obstacle);
			goal_state = OptTrajectory->get_optimal_trajectory_state(index_Add_final_checkpoint);


			referenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, next_id, Safety_Behaviour, Safety_Behaviour, 10));
			nextReferenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, next_id, Safety_Behaviour, Safety_Behaviour, 10));

			next_id++;
		}*/
	}
	else
	{
		cout << "Generate mission: (RacingMode) " << endl << endl;
		cout << "state_of_car: " << "xpos: " << egoState.x << " ypos: " << egoState.y << endl;

		//Set starting Checkpoint as egostate.
		referenceWaypoints[0].push_back(WayPoint(egoState.x, egoState.y, egoState.v, egoState.heading, 0, Racing_Behaviour, Racing_Behaviour, 10));

		//Find nearest point on opt.traj. wrt distance from center of egostate(the car).
		int index_ego_state = OptTrajectory->find_index(egoState.x,egoState.y);


		cout << "index_ego_state: " << index_ego_state << endl;
		cout << "state_near_ego_state: " << "xpos: " << OptTrajectory->get_optimal_trajectory_state(index_ego_state).x << " ypos: " << OptTrajectory->get_optimal_trajectory_state(index_ego_state).y << endl;

		//Find nearest point on opt.traj. wrt distance from center of the obstacle.
		int index_rect_obstacle_state = OptTrajectory->find_index(rectObstacle.pos.x,rectObstacle.pos.y);
		cout << endl << "state_of_obstacle: " << "xpos: " << rectObstacle.pos.x << " ypos: " << rectObstacle.pos.y << endl;

		cout << "state_near_obstacle_state: " << "xpos: " << egoState.x << " ypos: " << egoState.y << endl;

		cout << "index_obst_state: " << index_rect_obstacle_state << endl;
		cout << "state_near_obst_state: " << "xpos: " << OptTrajectory->get_optimal_trajectory_state(index_rect_obstacle_state).x << " ypos: " << OptTrajectory->get_optimal_trajectory_state(index_rect_obstacle_state).y << endl;

		//Calc distance from index_ego_state to index_rect_obstacle_state
		float length_to_middle_of_obstacle = OptTrajectory->integration_from_to(index_ego_state, index_rect_obstacle_state);

		if(length_to_middle_of_obstacle > 50.0)
			length_to_middle_of_obstacle=50.0;
		else if(length_to_middle_of_obstacle<30.0)
			length_to_middle_of_obstacle=30.0;

		cout << endl << "Length from car to obstacle intersection: " << length_to_middle_of_obstacle << endl;

		//Add length_to_middle_of_obstacle from state_near_obst_state along optimal trajectory as good way-point position.
		int index_final_checkpoint = OptTrajectory->get_index_from_state_add_length(index_rect_obstacle_state, length_to_middle_of_obstacle);
		State goal_state = OptTrajectory->get_optimal_trajectory_state(index_final_checkpoint);
		cout << "index_final_goal_mission: " << index_final_checkpoint << endl;

		cout << "state_of_goal_mission: " << "xpos: " << goal_state.x << " ypos: " << goal_state.y << "angle: "<< goal_state.heading << endl;

		//Push back calc state.
		referenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, 1,Racing_Behaviour, Racing_Behaviour, 10));
		nextReferenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v,goal_state.heading,1,Racing_Behaviour, Racing_Behaviour, 10));

		int index_Add_final_checkpoint = OptTrajectory->get_index_from_state_add_length(index_final_checkpoint, length_to_middle_of_obstacle);
		goal_state = OptTrajectory->get_optimal_trajectory_state(index_Add_final_checkpoint);

		referenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading,2,Racing_Behaviour, Racing_Behaviour, 10));
		nextReferenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y,egoState.v,goal_state.heading,2,Racing_Behaviour, Racing_Behaviour, 10));

		/*int i = index_Add_final_checkpoint + 20;
		int next_id = 3;
		for (i; i<(OptTrajectory->get_optimal_vector_size() - 35); i += 35)
		{
			int index_Add_final_checkpoint = OptTrajectory->get_index_from_state_add_length(i, length_to_middle_of_obstacle);
			goal_state = OptTrajectory->get_optimal_trajectory_state(index_Add_final_checkpoint);


			referenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, next_id, Racing_Behaviour, Racing_Behaviour, 10));
			nextReferenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, next_id, Racing_Behaviour, Racing_Behaviour, 10));

			next_id++;
		}*/
	}

	

	/*referenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, 2, Safety_Behaviour, Safety_Behaviour, 10));
	nextReferenceWaypoints[0].push_back(WayPoint(goal_state.x, goal_state.y, egoState.v, goal_state.heading, 2, Safety_Behaviour, Safety_Behaviour, 10));
	*/

	
}