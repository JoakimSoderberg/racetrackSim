/* Authors: Niclas Evestedt, Oskar Ljungqvist and Kristin Bergstrand 2014.

 This is a development project and a testing enviroment => there is no connection to RACETRACK WHAT SO EVER.

 To run this project alwas build release. Then run .exe file with command Ctrl + F5 this is the same as go to release folder and run .exe.

 A GUI is now displayed and press "s" to start.

 There is different visualization options, scope these with right mouse button on the GUI widow.

 Right click GUI: Draw mode/Draw Trajecories displays the whole tree structure.

 /Oskar

 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include <time.h>
#include "GlutVizualisation.h"

using namespace std;

//Safe mode = true
//Safe mode = false => racing mode
bool safe_mode = false;

int main(int argc, char **argv)
{
	Simulator       *simulator;
	MissionHandler  *missionHandler;
	Planner         *planner;
	MapHandler      *mapHandler;
	ObstacleMap		*obstacleMap;
	glutVizualisation *vizualiser;


	double deltaT = 0.04;	// Time to expand_tree() between updates from server
	double Ts = 0.02;		// Time between states recieved from simulation

	//Used to store best traj. and send this to server after packing.
	list<ReferenceTrajectoryState> reference_trajectory;

	/***********************************************************************************************/
	/*** Wait for valid position before initzialising the planner and obstacle map *****************/
	/**  RIGHT NOW WE HAVE TO SEND A VALID OBSTACLE ************************************************/
	/***********************************************************************************************/

	// Development file => we have to initiate states and obstacles.
	//x , y , vel, acc, heading, steerAngle, steerAngleRate.
	State egoState(10, -20, 15, 0, 0, 0, 0);

	//x,y,width,height,orientation,speed x, speed y.
	RectObstacle rectObstacle = RectObstacle(Point2D(60, -40), 25, 25, 0, 0.02, -0.01);

	obstacleMap = new ObstacleMap();

	obstacleMap->initialize();

	obstacleMap->insert_obstacle(rectObstacle);

	/****************************************************************/
	/*** Set up a simulator  ****************************************/
	/****************************************************************/

	simulator = new Simulator(deltaT);

	simulator->getSimulationModel()->set_state(egoState);

	simulator->set_obstacle_map(obstacleMap);


	/***************************************************************/
	/*** Setup for the Mission handler *****************************/
	/***************************************************************/
	missionHandler = new MissionHandler("ref.txt");
	// missionHandler->set_obastacle_map(obstacleMap);


	// missionHandler->set_mission_plan(safe_mode, egoState, rectObstacle);


	/****************************************************************/
	/*** Setup for Map Handler **************************************/
	/****************************************************************/

	mapHandler = new MapHandler(obstacleMap);
	mapHandler->setEgoState(&simulator->getSimulationModel()->state);


	/****************************************************************/
	/*** Setup for planner ******************************************/
	/****************************************************************/
	planner = new Planner(Ts);
	/*
	//Set the model used for forward propagation
	Model* planningModel = new Model(2);

	planningModel->set_state(egoState);

	planner->setModel(planningModel);

	//Set the obstacle map used for collission checking
	planner->setMapHandler(mapHandler);

	//set the state pointer to point at the latest state
	planner->setEgoState(&egoState);

	//set the global mission trajectory
	planner->setGlobalReferencePath(missionHandler->getMissionPlan(1), missionHandler->getNextMissionPlan(1));

	//initilaze the planner
	planner->initialize();

	/*

	/****************************************************************/
	/*** Setup for vizualisation ************************************/
	/****************************************************************/

	vizualiser = new glutVizualisation();
	vizualiser->setSimulationModel(simulator->getSimulationModel());
	vizualiser->setEgoState(egoState);
	vizualiser->setObstacleMap(obstacleMap);
	vizualiser->setPlanner(planner);
	vizualiser->setDrawOffset(egoState.x, egoState.y);
	vizualiser->setMissionHandler(missionHandler);

	vizualiser->initVizualisation();

	/****************************************************************/
	/*** Main execution loop     ************************************/
	/****************************************************************/
	while (true)
	{
		//Wait for go (press S)
		if (vizualiser->getRunState())
		{
			simulator->getSimulationModel()->step(Ts, 0, 0);
			egoState = simulator->getSimulationModel()->state;
		}
		//Uppdate grafic window.
		vizualiser->reDisplay();
	}
}

