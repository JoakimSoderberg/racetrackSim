#include "Model.h"
using namespace std;

/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/

/****************************************************************/
/*** Constructor Destructor  ************************************/
/****************************************************************/
Model::Model(int ID)
{
	debugLogg = false;
	state = State();
	parameters = Parameters();
	controller = new PurePursuitController();
	modelID = ID;
	controller->setState(&state);
	controller->setParameters(&parameters);
}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void	Model::set_state(double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in)
{
	state = State(x_in, y_in, v_in, a_in, heading_in, steerAngle_in, steerAngleRate_in);
}

void	Model::set_state(State& stateIn)
{
	state = stateIn;
}

void	Model::set_parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in)
{
	parameters = Parameters(L_in, vCH_in, Td_in, Ta_in, maxSteerAngle_in, maxSteerAngleRate_in, maxAccelearion_in, maxDeceleration_in);
	controller->setParameters(&parameters);
}

/****************************************************************/
/*** Evaluation methods *****************************************/
/****************************************************************/
double	Model::evaluateEuclidianCostToGo(State& stateFromIn, State& stateTowardsIn)
{
	//Calculate the cost of traveling from start to goal (euclidian distance)
	//cout << "Headings: " << stateFromIn.heading << " " << stateTowardsIn.heading << "Error heading: " 
	//	<< stateFromIn.heading-stateTowardsIn.heading << endl;

	return sqrt((stateFromIn.x - stateTowardsIn.x)*(stateFromIn.x - stateTowardsIn.x) + (stateFromIn.y - stateTowardsIn.y)*(stateFromIn.y - stateTowardsIn.y));
}

double	Model::evaluateExtentionCost(State &stateFromIn, Sample &sampleTowardsIn)
{
	double lookAheadDistance = 10;
	if (sqrt((stateFromIn.x - sampleTowardsIn.samplePoint.x)*(stateFromIn.x - sampleTowardsIn.samplePoint.x)
		+ (stateFromIn.y - sampleTowardsIn.samplePoint.y, 2)*(stateFromIn.y - sampleTowardsIn.samplePoint.y, 2)) <= lookAheadDistance)
	{
		return 10000;
	}


	//Calculate the dubin cost of traveling from start to end
	double minTurningRadius = 30;

	//set origin in stateFromIn
	double x = cos(stateFromIn.heading)*(sampleTowardsIn.samplePoint.x - stateFromIn.x) - sin(stateFromIn.heading)*(sampleTowardsIn.samplePoint.y - stateFromIn.y);
	double y = abs(sin(stateFromIn.heading)*(sampleTowardsIn.samplePoint.x - stateFromIn.x) + cos(stateFromIn.heading)*(sampleTowardsIn.samplePoint.y - stateFromIn.y));


	if (sqrt(x*x + (y - minTurningRadius)*(y - minTurningRadius)) < minTurningRadius)
	{
		/*double dc = sqrt(x*x + (y - minTurningRadius)*(y - minTurningRadius));
		double df = sqrt(x*x + (y + minTurningRadius)*(y + minTurningRadius));
		double alpha = acos((5 * minTurningRadius*minTurningRadius - df*df) / (4 * minTurningRadius*minTurningRadius));
		double thetac = atan2(x, minTurningRadius - y);
		if (thetac < 0)
			thetac += 2 * M_PI;

		double temo = minTurningRadius*(2 * M_PI - alpha + asin(dc*sin(thetac) / df) + asin(minTurningRadius*sin(alpha) / df));
		return minTurningRadius*(2 * M_PI - alpha + asin(dc*sin(thetac) / df) + asin(minTurningRadius*sin(alpha) / df));*/
		return 10000;
	}
	else
	{
		double dc = sqrt(x*x + (y - minTurningRadius)*(y - minTurningRadius));


		double thetac = atan2(x, minTurningRadius - y);
		if (thetac < 0)
			thetac += 2 * M_PI;

		double temo = sqrt(dc*dc - minTurningRadius*minTurningRadius) + minTurningRadius*(thetac - acos(minTurningRadius / dc));
		return sqrt(dc*dc - minTurningRadius*minTurningRadius) + minTurningRadius*(thetac - acos(minTurningRadius / dc));
	}
}

/****************************************************************/
/*** Simulation methods *****************************************/
/****************************************************************/
void	Model::step(double Ts, double steerAngleCommand, double accelerationCommand)
{
	/****************************************************************/
	/****** Propagate new states  ***********************************/
	/****************************************************************/
	state.x = state.x + Ts * state.v * cos(state.heading);
	state.y = state.y + Ts * state.v * sin(state.heading);

	double Gslip = 1 / (1 + state.v / parameters.vCH);
	state.heading = state.heading + Ts * state.v / parameters.L * tan(state.steerAngle) * Gslip;

	/****************************************************************/
	/****** Angle propagation with restrictions  ********************/
	/****************************************************************/

	double steerAngleRate = 1 / parameters.Td * (steerAngleCommand - state.steerAngle);

	if (abs(steerAngleRate) > parameters.maxSteerAngleRate)
	{
		if (steerAngleRate < 0)
			steerAngleRate = -parameters.maxSteerAngleRate;
		else
			steerAngleRate = parameters.maxSteerAngleRate;
	}

	state.steerAngleRate = steerAngleRate;

	state.steerAngle = state.steerAngle + Ts * steerAngleRate;

	if (abs(state.steerAngle) > parameters.maxSteerAngle)
	{

		if (state.steerAngle < 0)
			state.steerAngle = -parameters.maxSteerAngle;
		else
			state.steerAngle = parameters.maxSteerAngle;
	}

	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	state.v = state.v + Ts*state.a;

	state.a = state.a + Ts / parameters.Ta * (accelerationCommand - state.a);

}

pathFeasability	Model::simulateTo(double Ts, Node& nodeFromIn, pair<Sample, Sample>& samplePair, list<Trajectory*>& trajectoryListOut, list<Point2D>& intermediateControlPoints, MapHandler& mapHandler)
{

	//MOVE THIS TO SOME PARAMETER INGS!!
	double timeBetweenIntermediateNodes = 10000.5;

	double start_time = 0;
	Node temp_node = nodeFromIn;

	while (!temp_node.isRoot())
	{
		temp_node = temp_node.getParent();
		int samples_between_nodes = temp_node.getTrajectory().stateList.size() + 1;

		start_time += Ts*samples_between_nodes;
	}

	Trajectory* intermediateTrajectory = new Trajectory();

	double distanceOnControlLine = 0;

	/****************************************************************/
	/*** Simulation setup *******************************************/
	/****************************************************************/

	//set simulation start state to start state
	state = nodeFromIn.getState();

	double costMultiplier;
	//set driving direction
	if (samplePair.first.direction == Direction::forwardDirection)
	{
		costMultiplier = 1;
		state.v = samplePair.first.velocity;
	}
	else if (samplePair.first.direction == Direction::reverseDirection)
	{
		costMultiplier = 5;
		state.v = -samplePair.first.velocity;
	}

	intermediateTrajectory->stateList.push_back(state);


	//create a local reference control with only start and end point
	list<pair<Point2D, double>> referenceControl_in;

	//add start way point
	pair<Point2D, double> temp;
	temp.first = nodeFromIn.getStateKey();
	temp.second = 0;
	referenceControl_in.push_back(temp);

	temp.first = samplePair.first.samplePoint;
	temp.second = 0;
	referenceControl_in.push_back(temp);


	//add end node if batch sampling was used
	if (samplePair.second.active)
	{
		temp.first = samplePair.second.samplePoint;
		temp.second = 0;
		referenceControl_in.push_back(temp);
	}

	//add list to controller
	controller->setReferenceControl(referenceControl_in);


	/****************************************************************/
	/*** Simulation loop ********************************************/
	/****************************************************************/
	int i = 0;
	double simulationTime = 0;
	ControlCommand c;
	double vRef;

	while (!controller->referenceControl.empty())
	{
		/****************************************************************/
		/*** Calculate steer angle control ******************************/
		/****************************************************************/
		if (!controller->calcControl(c, vRef) || i == 1000)
		{
			//control failed reset lists and return
			intermediateControlPoints.clear();
			while (!trajectoryListOut.empty())
			{
				delete trajectoryListOut.front();
				trajectoryListOut.pop_front();
			}

			trajectoryListOut.clear();

			return nonFeasable;

		}

		/****************************************************************/
		/*** Simulate system with the control input *********************/
		/****************************************************************/
		//do not step when the last wp has been reached.
		if (!controller->nextControlWpEvent() && !controller->referenceControl.empty())
			step(Ts, c.steerAngleCommand, 0);

		/****************************************************************/
		/*** Perform collision checks with the map **********************/
		/**************************i**************************************/

		if (mapHandler.getCost(state, parameters, simulationTime + start_time) > 1)
		{
			if (trajectoryListOut.empty())
				return nonFeasable;
			else
			{
				trajectoryListOut.push_back(intermediateTrajectory);
				return partlyFeasable;
			}
		}

		/****************************************************************/
		/*** Update cost and add trajectory states **********************/
		/****************************************************************/
		intermediateTrajectory->cost += costMultiplier*sqrt((state.x - intermediateTrajectory->stateList.back().x)*(state.x - intermediateTrajectory->stateList.back().x) + (state.y - intermediateTrajectory->stateList.back().y)*(state.y - intermediateTrajectory->stateList.back().y));
		intermediateTrajectory->stateList.push_back(state);

		/****************************************************************/
		/*** Create an intermediate node on the control line ************/
		/****************************************************************/
		//Only add sample if batch is active and middle waypoint is taken
		if (controller->nextControlWpEvent() && controller->referenceControl.size() == 1 && samplePair.second.active)
		{
			intermediateControlPoints.push_back(samplePair.first.samplePoint);
			trajectoryListOut.push_back(intermediateTrajectory);

			//create a new trajectory for the next node
			intermediateTrajectory = new Trajectory();
			intermediateTrajectory->stateList.push_back(state);

		}
		else if (controller->isOkAddNewIntermediateNode() && i > 0)
		{

			intermediateControlPoints.push_back(controller->lookAheadPoints[0]);
			trajectoryListOut.push_back(intermediateTrajectory);

			//create a new trajectory for the next node
			intermediateTrajectory = new Trajectory();
			intermediateTrajectory->stateList.push_back(state);

		}


		simulationTime += Ts;
		i++;
	}


	/****************************************************************/
	/*** If program reached here everything went well ***************/
	/****************************************************************/
	//DEBUG!!!!!
	if (intermediateTrajectory->cost == 0)
		intermediateTrajectory->cost = 0.4;


	// add last part of trajectory to list and return
	trajectoryListOut.push_back(intermediateTrajectory);

	return feasable;
}

/****************************************************************/
/*** Misc  ******************************************************/
/****************************************************************/
bool Model::targetReached(State &stateIn, WayPoint &goalPoint)
{

	/****************************************************************/
	/*** Check if target is within limits of goal point *************/
	/****************************************************************/
	//calculate distance between goal point and current state
	double dist = sqrt((stateIn.x - goalPoint.wp.x)*(stateIn.x - goalPoint.wp.x) + (stateIn.y - goalPoint.wp.y)*(stateIn.y - goalPoint.wp.y));

	/****************************************************************/
	/*** Position contrained: Check only distance *******************/
	/****************************************************************/
	if (goalPoint.type == positionConstrained)
	{
		double visitedDistanceRadius = 4.9;
		if (dist < visitedDistanceRadius)
			return true;
		else
			return false;
	}
	/****************************************************************/
	/*** Heading contrained: Check distance and heading *************/
	/****************************************************************/
	else if (goalPoint.type == headingConstrained)
	{
		double visitedDistanceRadius = 3.8;
		double visitedHeadingLimit = 4.8*M_PI / 180;   //Maybe lower heading constraind for CDIO app. 1 from start

		double errorAngle = stateIn.heading - goalPoint.heading;

		while (errorAngle < -2 * M_PI)
			errorAngle += 2 * M_PI;

		while (errorAngle >2 * M_PI)
			errorAngle -= 2 * M_PI;


		if (dist < visitedDistanceRadius && abs(errorAngle) <= visitedHeadingLimit)
			return true;
		else
			return false;
	}
	else
		// Should not be here
		return false;
}