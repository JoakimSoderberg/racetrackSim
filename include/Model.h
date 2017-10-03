#pragma once
#define _USE_MATH_DEFINES

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <random>

#include "ModelParameters.h"
#include "Geometry.h"
#include "PlannerTypes.h"
#include "MapHandler.h"
#include "PurePursuitController.h"

using namespace std;

/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/
class Model 
{

	/****************************************************************/
	/*** Setup of random number generator ***************************/
	/****************************************************************/
	default_random_engine generator;
	normal_distribution<double> stdGaussianDist;
	
	
	public:
		bool debugLogg;
		int modelID;

		//current state of the system
        State		state;

		//parameter setup for the model
		Parameters	parameters;

		//handle to a control class used to caluclate control inputs
        PurePursuitController	*controller;

		/****************************************************************/
		/*** Constructor Destructor  ************************************/
		/****************************************************************/
		Model (int ID);
    
		/****************************************************************/
		/*** Set methods ************************************************/
		/****************************************************************/
		void set_state(double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in);
		void set_state(State& stateIn);
		void set_parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in);
		
		/****************************************************************/
		/*** Evaluation methods *****************************************/
		/****************************************************************/
		double evaluateExtentionCost(State &stateFromIn, Sample &sampleTowardsIn);
		double evaluateEuclidianCostToGo(State& stateFromIn, State& stateTowardsIn);

		/****************************************************************/
		/*** System methods *********************************************/
		/****************************************************************/
		void			 step(double Ts, double steerAngleCommand, double accelerationCommand);
		pathFeasability  simulateTo(double Ts, Node& nodeFromIn, pair<Sample, Sample>& samplePair, list<Trajectory*>& trajectoryListOut, list<Point2D>& intermediateControlPoints, MapHandler& mapHandler);

		/****************************************************************/
		/*** Misc *******************************************************/
		/****************************************************************/
		bool targetReached(State &stateIn, WayPoint &goalState);
		
};
