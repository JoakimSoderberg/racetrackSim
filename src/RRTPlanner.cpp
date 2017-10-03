#include <algorithm>    // std::sort
#include "RRTPlanner.h"
#include <fstream>
#include <iostream>

/****************************************************************/
/*** Constructor destructor *************************************/
/****************************************************************/
Planner::Planner(double Ts_in)
{
	samplingStategies = new SamplingStrategies();

	lowerBoundCost = DBL_MAX;
	lowerBoundNode = NULL;
	nextLowerBoundNode = NULL;
	nextLowerBoundCost = DBL_MAX;

	root = NULL;

	numNodes = 0;
	numForwardNodes = 0;
	numReverseNodes = 0;

	maxConnectionTries = 10;

	goalPoint = NULL;
	nextGoalPoint = NULL;

	simulationModel = NULL;

	Ts = Ts_in;
}

Planner::~Planner()
{
	// Delete all forward nodes
	for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
		delete *iter;

	// Delete all reverse nodes
	for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
		delete *iter;

	// Delete all safe nodes
	for (list<Node*>::iterator iter = stoppingNodeList.begin(); iter != stoppingNodeList.end(); iter++)
		delete *iter;

}

/****************************************************************/
/*** Compare methods used for sort  *****************************/
/****************************************************************/
int		compareNodeCostPairs(pair<Node*, double> i, pair<Node*, double> j) {

	return (i.second < j.second);
}

int		compareLowerBoundCost(Node* i, Node* j) {

	return (i->getLowerBoundCost() < j->getLowerBoundCost());
}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void	Planner::setModel(Model* simulationModelIn)
{

	if (simulationModel)
		delete simulationModel;

	simulationModel = simulationModelIn;

	// Initialize the root vertex
	root = new Node;
	//root->state = new State(simulationModel->state);
	root->setState(simulationModel->state);

	root->costFromParent = 0.0;
	root->costFromRoot = 0.0;
	Point2D stateKey;
	stateKey.x = cos(root->getState().heading)*simulationModel->controller->lookAheadDistance + root->getState().x;
	stateKey.y = sin(root->getState().heading)*simulationModel->controller->lookAheadDistance + root->getState().y;
	root->stateKey = stateKey;
	root->parent = NULL;
	root->root = true;
}

void	Planner::setEgoState(State* const stateIn)
{
	egoState = stateIn;
}

void	Planner::setGoal(WayPoint* goalPointIn)
{
	goalPoint = goalPointIn;
}

void	Planner::setNextGoal(WayPoint* nextGoalPointIn)
{
	nextGoalPoint = nextGoalPointIn;
}

void	Planner::setGlobalReferencePath(list<WayPoint> globalReference_in, list<WayPoint> globalNextReference_in)
{
	globalNextReferencePath = globalNextReference_in;
	globalReferencePath = globalReference_in;
	goalPoint = &globalReference_in.front();
	nextGoalPoint = &globalNextReference_in.front();
}

void Planner::setNewRootNode(Node& newRoot)
{
	list<Node*> tempForwardNodeList;
	list<Node*> tempReverseNodeList;
	list<Node*> tempStoppingNodeList;

	markForDeletion(newRoot);

	for (list<Node*>::iterator iter = stoppingNodeList.begin(); iter != stoppingNodeList.end(); iter++)
	{
		Node* node = *iter;
		if (node->markedToKeep)
			tempStoppingNodeList.push_back(node);

	}

	for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
	{
		Node* node = *iter;
		if (!node->markedToKeep)
		{
			delete *iter;
			numNodes--;
			numForwardNodes--;
		}
		else
		{
			node->markedToKeep = false;
			tempForwardNodeList.push_back(node);
		}
	}

	for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
	{
		Node* node = *iter;
		if (!node->markedToKeep)
		{
			delete *iter;
			numNodes--;
			numReverseNodes--;
		}
		else
		{
			node->markedToKeep = false;
			tempReverseNodeList.push_back(node);
		}
	}

	forwardNodeList.clear();
	reverseNodeList.clear();
	stoppingNodeList.clear();

	forwardNodeList = tempForwardNodeList;
	reverseNodeList = tempReverseNodeList;
	stoppingNodeList = tempStoppingNodeList;

	newRoot.root = true;
	root = &newRoot;

}

void Planner::markForDeletion(Node& keepFromNode)
{
	set<Node*>::iterator n;
	for (n = keepFromNode.children.begin(); n != keepFromNode.children.end(); n++)
	{
		Node* child = *n;

		markForDeletion(*child);

		child->markedToKeep = true;

	}

	keepFromNode.markedToKeep = true;
}


void	Planner::setMapHandler(MapHandler* mapHandlerIn)
{
	mapHandler = mapHandlerIn;
}

/****************************************************************/
/*** Get methods ************************************************/
/****************************************************************/
Node&	Planner::getRootNode(){

	return *root;
}

int	Planner::getBestControl(list<pair<Point2D, double>>& controlOut){

	Node* currentNode;
	pair<Point2D, double> controlPair;



	lazy_check();
	lastBestSequence.clear();
	nextLastBestSequence.clear();

	if (nextLowerBoundNode != NULL)
	{
		currentNode = nextLowerBoundNode;
	}
	else
	{
		if (lowerBoundNode == NULL && stoppingNodeList.empty())
		{
			return 0;
		}
		else if (lowerBoundNode == NULL)
		{
			stoppingNodeList.sort(compareLowerBoundCost);
			currentNode = stoppingNodeList.front();
		}
		else
		{
			currentNode = lowerBoundNode;
		}
	}


	if (currentNode == nextLowerBoundNode)
	{
		while (!currentNode->isRoot())
		{
			nextLastBestSequence.push_front(currentNode);

			controlPair.first = currentNode->getStateKey();
			//TODO fix this
			if (currentNode->getDirection() == forwardDirection)
				controlPair.second = currentNode->getVelocityRef();
			else
				controlPair.second = -currentNode->getVelocityRef();

			controlOut.push_front(controlPair);

			currentNode = &currentNode->getParent();
		}
	}
	else if (currentNode == lowerBoundNode)
	{
		while (!currentNode->isRoot())
		{
			lastBestSequence.push_front(currentNode);

			controlPair.first = currentNode->getStateKey();
			//TODO fix this
			if (currentNode->getDirection() == forwardDirection)
				controlPair.second = currentNode->getVelocityRef();
			else
				controlPair.second = -currentNode->getVelocityRef();

			controlOut.push_front(controlPair);

			currentNode = &currentNode->getParent();
		}
	}

	//add the root to the control
	controlPair.first = currentNode->getStateKey();
	//TODO fix this
	if (currentNode->getDirection() == forwardDirection)
		controlPair.second = currentNode->getVelocityRef();
	else
		controlPair.second = -currentNode->getVelocityRef();

	controlOut.push_front(controlPair);

	lastBestSequence.push_front(currentNode);
	nextLastBestSequence.push_front(currentNode);

	return 1;
}


/****************************************************************/
/*** Misc *******************************************************/
/****************************************************************/

double Planner::time_from_node_to_root(Node* start_node){

	double time = 0;
	Node* currentNode = start_node;

	while (!currentNode->isRoot())
	{
		currentNode = &currentNode->getParent();
		int steps_between_nodes = currentNode->getTrajectory().stateList.size() - 1;
		time += Ts*steps_between_nodes;
	}
	time += Ts; //NOT REALLY SURE ABOUT THIS.
	return time;
}

bool Planner::check_lower_bound_node(){

	bool collision = false;
	Node* currentNode = lowerBoundNode;

	//Check if trajecoty from root to lowerBoundNode is in colusion.
	while (!currentNode->isRoot())
	{
		double time = time_from_node_to_root(currentNode);
		if (mapHandler->getCostDynamic(currentNode->getState(), simulationModel->parameters, time) > 1)
		{
			collision = true;
			return collision;
		}

		list<State> temp_state_list = currentNode->getTrajectory().stateList;

		temp_state_list.pop_front(); //Last is same as nextnode.
		temp_state_list.pop_back();  //First is same as node.

		while (!temp_state_list.empty())
		{
			time -= Ts;
			if (mapHandler->getCostDynamic(temp_state_list.back(), simulationModel->parameters, time) > 1)
			{
				collision = true;
				return collision;
			}
			temp_state_list.pop_back();
		}

		currentNode = &currentNode->getParent();
	}

	return collision;
}
bool Planner::check_next_lower_bound_node(){

	bool collision = false;
	Node* currentNode = nextLowerBoundNode;

	//Check if trajecoty from root to lowerBoundNode is in colusion.
	while (!currentNode->isRoot())
	{
		double time = time_from_node_to_root(currentNode);
		if (mapHandler->getCostDynamic(currentNode->getState(), simulationModel->parameters, time) > 1)
		{
			collision = true;
			return collision;
		}

		list<State> temp_state_list = currentNode->getTrajectory().stateList;

		temp_state_list.pop_front(); //Last is same as nextnode.
		temp_state_list.pop_back();  //First is same as node.

		while (!temp_state_list.empty())
		{
			time -= Ts;
			if (mapHandler->getCostDynamic(temp_state_list.back(), simulationModel->parameters, time) > 1)
			{
				collision = true;
				return collision;
			}
			temp_state_list.pop_back();
		}

		currentNode = &currentNode->getParent();
	}

	return collision;
}

void Planner::lazy_check(){

	//Check nextLowerBoundNode
	while (nextLowerBoundNode != NULL)
	{
		//Collision?
		if (check_next_lower_bound_node())
		{
			cout << "collision!" << endl;
		}
		else return;

		//If collision pick next best as lower bound node. And reloop
		if (!nextLastBestSequence.empty() && simulationModel->targetReached(nextLastBestSequence.front()->getState(), *nextGoalPoint))
		{
			nextLowerBoundNode = nextLastBestSequence.front();
			nextLastBestSequence.pop_front();
		}
		else
			nextLowerBoundNode = NULL;
	}

	//Check lowerBoundNode
	while (lowerBoundNode != NULL)
	{
		//Collision?
		if (check_lower_bound_node())
		{
			cout << "collision!" << endl;
		}
		else return;

		//If collision pick next best as lower bound node. And reloop
		if (!lastBestSequence.empty() && simulationModel->targetReached(lastBestSequence.front()->getState(), *goalPoint))
		{
			lowerBoundNode = lastBestSequence.front();
			lastBestSequence.pop_front();
		}
		else
			lowerBoundNode = NULL;
	}
}

list<ReferenceTrajectoryState> Planner::calculate_reference_trajectory(double Ts){

	list<ReferenceTrajectoryState> temp_ref_traj;

	list<State> all_states;
	list<State> temp_states;
	State state_node;
	Node* currentNode = lowerBoundNode;

	//Sort all states from root to lowerBoundNode
	while (!currentNode->isRoot())
	{
		temp_states = currentNode->getTrajectory().stateList;
		if (!temp_states.empty())
			temp_states.pop_front();
		else
			break;

		for (list<State>::reverse_iterator iter = temp_states.rbegin(); iter != temp_states.rend(); ++iter)
		{
			//Transform to OSAAR2013 coordinate system
			iter->x = -iter->x / 100;
			iter->y = -iter->y / 100;
			iter->heading = iter->heading - M_PI;
			all_states.push_front(*iter);
		}

		currentNode = &currentNode->getParent();
	}

	if (currentNode->isRoot())
	{
		temp_states = currentNode->getTrajectory().stateList;

		for (list<State>::reverse_iterator iter = temp_states.rbegin(); iter != temp_states.rend(); ++iter)
		{
			//Transform to OSAAR2013 coordinate system
			iter->x = -iter->x / 100;
			iter->y = -iter->y / 100;
			iter->heading = iter->heading - M_PI;
			all_states.push_front(*iter);
		}
	}

	ReferenceTrajectoryState temp_ref_traj_state;

	for (list < State > ::iterator iter = all_states.begin(); iter != all_states.end(); iter++)
	{
		temp_ref_traj_state.x = iter->x;
		temp_ref_traj_state.y = iter->y;
		temp_ref_traj_state.theta = iter->heading;
		temp_ref_traj.push_back(temp_ref_traj_state);
	}

	return temp_ref_traj;
}

void	Planner::calculateHeuristicCostToStoppingNodes(Sample &sampleIn, list<Node*> &nodeList, vector<pair<Node*, double>> &nodeCostPairVector, bool optimization){

	pair<Node*, double> costPair;
	for (list<Node*>::iterator iter = nodeList.begin(); iter != nodeList.end(); iter++)
	{
		//connect to stopping nodes only if direction changes
		if ((*iter)->getDirection() != sampleIn.direction)
		{
			costPair.first = *iter;

			double trajCost = simulationModel->evaluateExtentionCost(((*iter)->state), sampleIn);

			if (optimization)
				costPair.second = (*iter)->costFromRoot + trajCost;
			else
				costPair.second = trajCost;

			nodeCostPairVector.push_back(costPair);
		}
	}
}

void	Planner::calculateHeuristicCost(Sample &sampleIn, list<Node*> &nodeList, vector<pair<Node*, double>> &nodeCostPairVector, bool optimization){

	pair<Node*, double> costPair;
	for (list<Node*>::iterator iter = nodeList.begin(); iter != nodeList.end(); iter++)
	{

		costPair.first = *iter;

		double trajCost = simulationModel->evaluateExtentionCost(((*iter)->state), sampleIn);

		if (optimization)
			costPair.second = (*iter)->costFromRoot + trajCost;
		else
			costPair.second = trajCost;

		nodeCostPairVector.push_back(costPair);

	}
}

void	Planner::propagateSafeState(Node& nodeIn){

	Node* currentNode = &nodeIn;
	while (!currentNode->isRoot() && !currentNode->isSafe())
	{
		currentNode->setSafe();
		currentNode = &currentNode->getParent();
	}
}

int		Planner::checkUpdateBestNode(Node& nodeIn){

	if (simulationModel->targetReached(nodeIn.getState(), *nextGoalPoint))
	{
		double Heading_cost = 5 * abs(nodeIn.getState().heading - nextGoalPoint->heading);
		double position_cost = 1 * ((nodeIn.getState().x - nextGoalPoint->wp.x)*(nodeIn.getState().x - nextGoalPoint->wp.x) +
			(nodeIn.getState().y - nextGoalPoint->wp.y)*(nodeIn.getState().y - nextGoalPoint->wp.y));

		double currentCost = nodeIn.getCost() + Heading_cost + position_cost;

		double max_allowed_cost = 1000;

		if (((nextLowerBoundNode == NULL) && currentCost < max_allowed_cost) || ((nextLowerBoundNode != NULL) && (currentCost < nextLowerBoundCost))) {

			nextLowerBoundNode = &nodeIn;
			nextLowerBoundCost = currentCost;
		}
	}
	else if (simulationModel->targetReached(nodeIn.getState(), *goalPoint))
	{
		double Heading_cost = 5 * abs(nodeIn.getState().heading - goalPoint->heading);
		double position_cost = 1 * ((nodeIn.getState().x - goalPoint->wp.x)*(nodeIn.getState().x - goalPoint->wp.x) +
			(nodeIn.getState().y - goalPoint->wp.y)*(nodeIn.getState().y - goalPoint->wp.y));

		double currentCost = nodeIn.getCost() + Heading_cost + position_cost;

		if ((lowerBoundNode == NULL) || ((lowerBoundNode != NULL) && (currentCost < lowerBoundCost))) {

			lowerBoundNode = &nodeIn;
			lowerBoundCost = currentCost;
		}
	}


	/****************************************************************/
	/*** Check if the goal has changed since last itteration  *******/
	/****************************************************************/
	if (goalPoint->ID != prevGoalPointID)
	{
		//if goal has changed reset the best node
		if (nextLowerBoundNode)
		{
			lowerBoundNode = nextLowerBoundNode;
			nextLowerBoundNode = NULL;
		}
		else if (lowerBoundNode)
		{
			lowerBoundNode = NULL;
		}

		prevGoalPointID = goalPoint->ID;
	}

	return 1;
}

bool	Planner::checkGlobalGoal()
{
	//get current waypoint
	if (!globalReferencePath.empty())
	{
		goalPoint = &globalReferencePath.front();
		if (!globalNextReferencePath.empty())
			nextGoalPoint = &globalNextReferencePath.front();
	}
	else
		return false;


	/****************************************************************/
	/*** check condition for leaving exit condition      ************/
	/****************************************************************/
	if (sqrt(pow(previousGoalPoint.wp.x - egoState->x, 2) + pow(previousGoalPoint.wp.y - egoState->y, 2)) > previousGoalPoint.exitDistance)
		//set behaviour to entry behaviour
		behaviourState = goalPoint->entryBehaviour;


	/****************************************************************/
	/*** check visited condition for position constrained waypoint **/
	/****************************************************************/
	if (goalPoint->type == positionConstrained)
	{

		double visitedDistanceRadius = 6;

		double distance_to_target = sqrt((goalPoint->wp.x - egoState->x)*(goalPoint->wp.x - egoState->x) +
			(goalPoint->wp.y - egoState->y)*(goalPoint->wp.y - egoState->y));

		double distance_to_next_target = sqrt((nextGoalPoint->wp.x - egoState->x)*(nextGoalPoint->wp.x - egoState->x) +
			(nextGoalPoint->wp.y - egoState->y)*(nextGoalPoint->wp.y - egoState->y));

		double distance_between_targets = sqrt((nextGoalPoint->wp.x - goalPoint->wp.x)*(nextGoalPoint->wp.x - goalPoint->wp.x) +
			(nextGoalPoint->wp.y - goalPoint->wp.y)*(nextGoalPoint->wp.y - goalPoint->wp.y));

		double visitedHeadingLimit = 7 * M_PI / 180;

		double errorAngle = egoState->heading - goalPoint->heading;

		while (errorAngle < -2 * M_PI)
			errorAngle += 2 * M_PI;

		while (errorAngle >2 * M_PI)
			errorAngle -= 2 * M_PI;

		//TODO
		//Last condition is put to solve a bug for the final two checkpoints.
		bool goal_reached = (distance_to_target < visitedDistanceRadius) || (distance_between_targets > distance_to_next_target && (distance_between_targets < 120));

		if (goal_reached)
		{
			//set behaviour to exit behaviour
			behaviourState = goalPoint->exitBehaviour;

			//set previous waypoint
			previousGoalPoint = *goalPoint;

			//add waypoint to visited list
			visitedWayPoints.push_back(*goalPoint);

			globalReferencePath.pop_front();

			globalNextReferencePath.pop_front();

			//if there are no more waypoints the mission is finished so return
			if (globalReferencePath.empty())
				return false;

			//set new globalGoal to the next waypoint
			goalPoint = &globalReferencePath.front();

			recalculateCostToGo();

			if (globalNextReferencePath.empty())
				return true;

			//set new globalNextGoal to the next waypoint
			nextGoalPoint = &globalNextReferencePath.front();
		}
	}

	/****************************************************************/
	/*** check visited condition for heading contrained waypoint  ***/
	/****************************************************************/
	else if (goalPoint->type == headingConstrained)
	{
		double visitedDistanceRadius = 5;
		double visitedHeadingLimit = 7 * M_PI / 180;

		double errorAngle = egoState->heading - goalPoint->heading;

		while (errorAngle < -2 * M_PI)
			errorAngle += 2 * M_PI;

		while (errorAngle >2 * M_PI)
			errorAngle -= 2 * M_PI;


		double dist = sqrt((goalPoint->wp.x - egoState->x)*(goalPoint->wp.x - egoState->x) + (goalPoint->wp.y - egoState->y)*(goalPoint->wp.y - egoState->y));

		bool goal_reached = dist < visitedDistanceRadius && abs(errorAngle) <= visitedHeadingLimit;

		if (goal_reached)
		{
			//set behaviour to exit behaviour
			behaviourState = goalPoint->exitBehaviour;

			//set previous waypoint
			previousGoalPoint = *goalPoint;

			//add waypoint to visited list
			visitedWayPoints.push_back(*goalPoint);
			globalReferencePath.pop_front();
			globalNextReferencePath.pop_front();

			//if there are no more waypoints the mission is finished so return
			if (globalReferencePath.empty())
				return false;

			//set new globalGoal to the next waypoint
			goalPoint = &globalReferencePath.front();

			//recaclulate new cost to go
			recalculateCostToGo();

			if (globalNextReferencePath.empty())
				return true;

			//set new globalNextGoal to the next waypoint
			nextGoalPoint = &globalNextReferencePath.front();
		}
	}

	return true;
}

void	Planner::recalculateCostToGo()
{
	/****************************************************************/
	/*** Recalculate Cost To Go when goal has changed  **************/
	/****************************************************************/


	for (list<Node*>::iterator currentNode = forwardNodeList.begin(); currentNode != forwardNodeList.end(); currentNode++)
	{
		(*currentNode)->upperBoundCost = numeric_limits<double>::max();

		State state(goalPoint->wp.x, goalPoint->wp.y, 0, 0, 0, 0, 0);
		(*currentNode)->lowerBoundCost = simulationModel->evaluateEuclidianCostToGo((*currentNode)->getState(), state);

	}

	for (list<Node*>::iterator currentNode = reverseNodeList.begin(); currentNode != reverseNodeList.end(); currentNode++)
	{
		(*currentNode)->upperBoundCost = numeric_limits<double>::max();

		State state(goalPoint->wp.x, goalPoint->wp.y, 0, 0, 0, 0, 0);
		(*currentNode)->lowerBoundCost = simulationModel->evaluateEuclidianCostToGo((*currentNode)->getState(), state);
	}
}

void	Planner::updateUpperBoundCost(Node& nodeIn)
{
	Node* currentNode = &nodeIn;

	while (!currentNode->isRoot() && currentNode->getParent().upperBoundCost > currentNode->upperBoundCost + currentNode->costFromParent)
	{
		currentNode->getParent().upperBoundCost = currentNode->upperBoundCost + currentNode->costFromParent;

		currentNode = &currentNode->getParent();
	}
}

void	Planner::debugPrintToFile()
{
	FILE * pFile;


	pFile = fopen("myfile.txt", "w");
	list<Trajectory*>	trajectoryListOut;
	list<Point2D>		intermediateControlPoints;

	Node* startNode = new Node();
	State state;
	startNode->setState(state);
	startNode->stateKey = Point2D(10, 0);
	startNode->root = true;

	fprintf(pFile, "test = [");
	for (int x = -100; x < 100; x += 1)
	{
		fprintf(pFile, ";\n");
		//double x = -16; double y =0;
		for (int y = -100; y <= 100; y += 1)
		{
			pair<Sample, Sample> goalState;
			goalState.first = Sample(Point2D(x, y), Direction::forwardDirection);

			if (simulationModel->simulateTo(Ts, *startNode, goalState, trajectoryListOut, intermediateControlPoints, *mapHandler))
			{
				//add the last sample point
				Node* newNode = insertNode(*startNode, goalState.first, *trajectoryListOut.back());
				trajectoryListOut.clear();
				newNode->getTrajectory().stateList.clear();
				fprintf(pFile, " %f,", newNode->getTrajectory().cost);
			}
		}
	}
	fprintf(pFile, "];");


	//fprintf (pFile, "");
	fclose(pFile);


}


/****************************************************************/
/*** Tree construction methods **********************************/
/****************************************************************/

pathFeasability Planner::seekTreeConnection(pair<Sample, Sample>& samplePair, Node*& bestNode, list<Trajectory*>& trajectoryListOut, list<Point2D>& intermediateControlPoints)
{
	/****************************************************************/
	/*** Setup varibles *********************************************/
	/****************************************************************/
	//function varibles
	pathFeasability feasabilityStatus = nonFeasable;

	//make sure vector gets correct length
	int numNodes;
	if (samplePair.first.direction == Direction::forwardDirection)
	{
		numNodes = numForwardNodes;
	}
	else if (samplePair.first.direction == Direction::reverseDirection)
	{
		numNodes = numReverseNodes;
	}

	vector<pair<Node*, double>> nodeCostPairVector;



	/****************************************************************/
	/*** Calculate cost to other nodes and sort accoring to least  **/
	/*** cost                                   *********************/
	/****************************************************************/

	//choose between exploration or optimization heuristics
	bool optimization = stdUniformDist(generator) < 0.7;

	// Compute the cost of extension for each near node
	if (samplePair.first.direction == Direction::forwardDirection)
		calculateHeuristicCost(samplePair.first, forwardNodeList, nodeCostPairVector, optimization);
	else if (samplePair.first.direction == Direction::reverseDirection)
		calculateHeuristicCost(samplePair.first, reverseNodeList, nodeCostPairVector, optimization);

	// See if there are possible conections in stopping nodes (This is only allowed at direction changes)
	calculateHeuristicCostToStoppingNodes(samplePair.first, stoppingNodeList, nodeCostPairVector, optimization);

	// Sort nodes according to cost
	std::sort(nodeCostPairVector.begin(), nodeCostPairVector.end(), compareNodeCostPairs);


	/****************************************************************/
	/*** Try connection to node by simulation ***********************/
	/****************************************************************/
	// Try out each extension according to increasing cost
	int numberOfTriedConnections = 0;
	bool connectionEstablished = false;
	for (vector<pair<Node*, double>>::iterator iter = nodeCostPairVector.begin(); iter != nodeCostPairVector.end(); iter++)
	{
		//Break if maximum connection tries has been reached
		numberOfTriedConnections++;
		if (numberOfTriedConnections > maxConnectionTries)
			break;

		Node* currentNode = iter->first;

		// Extend the current node towards stateIn (and this time check for collision with obstacles)
		feasabilityStatus = simulationModel->simulateTo(Ts, *currentNode, samplePair, trajectoryListOut, intermediateControlPoints, *mapHandler);

		if (feasabilityStatus == feasable || feasabilityStatus == partlyFeasable)
		{
			bestNode = currentNode;
			break;
		}
	}

	//return feasability type
	return feasabilityStatus;


}

pathFeasability Planner::seekGoalConnection(pair<Sample, Sample>& goalSamplePair, Node& startNode, list<Trajectory*>& trajectoryListOut, list<Point2D>& intermediateControlPoints)
{
	//function varibles
	pathFeasability feasabilityStatus;

	// Extend the current node towards stateIn (and this time check for collision with obstacles)
	feasabilityStatus = simulationModel->simulateTo(Ts, startNode, goalSamplePair, trajectoryListOut, intermediateControlPoints, *mapHandler);

	return feasabilityStatus;

}

Node*	Planner::insertNode(Node& nodeStartIn, Sample &randomSample, Trajectory& trajectoryIn)
{

	// Create new node
	Node* nodeNew = new Node;
	//nodeNew->state		= new State;
	nodeNew->stateKey = randomSample.samplePoint;
	nodeNew->parent = NULL;
	nodeNew->getState() = trajectoryIn.getEndState();
	nodeNew->direction = randomSample.direction;
	nodeNew->vRef = randomSample.velocity;

	if (randomSample.direction == Direction::forwardDirection)
	{
		this->forwardNodeList.push_front(nodeNew);
		this->numForwardNodes++;
	}
	else if (randomSample.direction == Direction::reverseDirection)
	{
		this->reverseNodeList.push_front(nodeNew);
		this->numReverseNodes++;
	}

	this->numNodes++;

	// Insert the trajectory between the start and end vertices
	insertNode(nodeStartIn, trajectoryIn, *nodeNew);

	return nodeNew;
}

int		Planner::insertNode(Node& nodeStartIn, Trajectory& trajectoryIn, Node& nodeEndIn) {

	// Update the costs
	nodeEndIn.costFromParent = trajectoryIn.evaluateCost();
	nodeEndIn.costFromRoot = nodeStartIn.costFromRoot + nodeEndIn.costFromParent;

	checkUpdateBestNode(nodeEndIn);

	// Update the trajectory between the two vertices
	//    if (nodeEndIn.trajFromParent)
	//        delete nodeEndIn.trajFromParent;
	nodeEndIn.trajFromParent = trajectoryIn;

	// Update the parent to the end vertex
	if (nodeEndIn.parent)
		nodeEndIn.parent->children.erase(&nodeEndIn);
	nodeEndIn.parent = &nodeStartIn;

	// Add the end vertex to the set of children
	nodeStartIn.children.insert(&nodeEndIn);

	return 1;
}

void Planner::insertIntermediateNodes(Node*& parentNode, list<Point2D>& intermediateControlPoints, list<Trajectory*>	trajectoryListOut, list<Node*> addedNodesOut, Direction direction, double vRef)
{

	/****************************************************************/
	/*** Add all intermediate to the tree ***************************/
	/****************************************************************/
	while (!intermediateControlPoints.empty())
	{
		/****************************************************************/
		/*** Varible setup **********************************************/
		/****************************************************************/
		Trajectory* currentTrajectory = trajectoryListOut.front();
		trajectoryListOut.pop_front();

		Point2D currentControlPoint = intermediateControlPoints.front();
		intermediateControlPoints.pop_front();


		/****************************************************************/
		/*** Add the node ***********************************************/
		/****************************************************************/
		Sample sample(currentControlPoint, direction, vRef);
		Node* newNode = insertNode(*parentNode, sample, *currentTrajectory);
		parentNode = newNode;
		if (newNode == NULL)
			return;

		//update lowerbound cost-to-go
		State state(goalPoint->wp.x, goalPoint->wp.y, 0, 0, 0, 0, 0);
		newNode->lowerBoundCost =
			simulationModel->evaluateEuclidianCostToGo(newNode->getState(), state);

		//add added nodes to the list
		addedNodesOut.push_back(newNode);
	}
}

int		Planner::deleteBranch(Node& nodeIn, Node& keepNode)
{

	if (&nodeIn != &keepNode)
		deleteChild(nodeIn, keepNode);

	if (nodeIn.getDirection() == Direction::forwardDirection)
	{
		forwardNodeList.remove(&nodeIn);
		stoppingNodeList.remove(&nodeIn);
		numForwardNodes--;
	}
	else if (nodeIn.getDirection() == Direction::reverseDirection)
	{
		reverseNodeList.remove(&nodeIn);
		stoppingNodeList.remove(&nodeIn);
		numReverseNodes--;
	}

	numNodes--;

	delete &nodeIn;

	return 0;

}

int		Planner::deleteChild(Node& nodeIn, Node& keepNode)
{
	set<Node*>::iterator iterator = nodeIn.children.begin();

	while (iterator != nodeIn.children.end())
	{
		// copy the current iterator then increment it
		set<Node*>::iterator current = iterator++;
		Node* node = *current;
		if (node != &keepNode)
		{
			deleteChild(*node, keepNode);
			nodeIn.children.erase(node);

			if (node->getDirection() == Direction::forwardDirection)
			{
				forwardNodeList.remove(node);
				stoppingNodeList.remove(node);
				numForwardNodes--;
			}
			else if (node->getDirection() == Direction::reverseDirection)
			{
				reverseNodeList.remove(node);
				stoppingNodeList.remove(node);
				numReverseNodes--;
			}

			numNodes--;

			delete node;
		}
	}

	return 0;
}

int		Planner::extendTo(pair<Sample, Sample> samplePair, pair<Sample, Sample> goalSamplePair)
{

	//Function varibles
	list<Trajectory*>	trajectoryListOut;
	list<Point2D>		intermediateControlPoints;

	//set the direction of this extension
	Direction extensionDirection = samplePair.first.direction;

	Node* parentNode = NULL;

	/****************************************************************/
	/*** Find best extentions in tree *******************************/
	/****************************************************************/
	pathFeasability sampleFeasbilityStatus = seekTreeConnection(samplePair, parentNode, trajectoryListOut, intermediateControlPoints);

	//add all nodes if path is feasable and only feasable part if path is partly feasable
	if (sampleFeasbilityStatus == feasable || sampleFeasbilityStatus == partlyFeasable)
	{

		list<Node*> tempAddedNodes;

		/****************************************************************/
		/*** Add new node and intermediate nodes to the tree  ***********/
		/****************************************************************/
		insertIntermediateNodes(parentNode, intermediateControlPoints, trajectoryListOut, tempAddedNodes, extensionDirection, samplePair.first.velocity);

		//only add sample node if the path is fully feasable
		if (sampleFeasbilityStatus == feasable)
		{
			//add the last sample point
			Sample activeSample;
			if (!samplePair.second.active)
				activeSample = samplePair.first;
			else
				activeSample = samplePair.second;

			Node* newNode = insertNode(*parentNode, activeSample, *trajectoryListOut.back());
			if (newNode == NULL)
				return 0;

			//plot info
			newNode->isSampleNode = true;

			//update lowerbound cost-to-go
			State state(goalPoint->wp.x, goalPoint->wp.y, 0, 0, 0, 0, 0);
			newNode->lowerBoundCost =
				simulationModel->evaluateEuclidianCostToGo(newNode->getState(), state);

			//add node to safe stopping set and mark intermediate nodes as safe
			stoppingNodeList.push_front(newNode);
			propagateSafeState(*newNode);

			//add last sample node for goal check
			tempAddedNodes.push_back(newNode);
		}

		// ****************************************************************/
		// *** Try connecting newly added nodes to goal   *****************/
		// ****************************************************************/

		Node*  goalNodeOut = NULL;

		while (!tempAddedNodes.empty())
		{

			Node* currentNode = tempAddedNodes.front();
			tempAddedNodes.pop_front();

			list<Trajectory*>	goalTrajectoryListOut;
			list<Point2D>		goalIntermediateControlPoints;


			pathFeasability goalFeasbilityStatus = seekGoalConnection(goalSamplePair, *currentNode, goalTrajectoryListOut, goalIntermediateControlPoints);

			//add all nodes if path is feasable and only feasable part if path is partly feasable
			if (goalFeasbilityStatus == feasable || goalFeasbilityStatus == partlyFeasable)
			{

				//****************************************************************
				//*** Add new node and intermediate nodes to the tree  ***********
				//****************************************************************
				parentNode = currentNode;

				list<Node*> noUse;
				insertIntermediateNodes(parentNode, goalIntermediateControlPoints, goalTrajectoryListOut, noUse, extensionDirection, goalSamplePair.first.velocity);

				//only add sample node if the path is fully feasable
				if (goalFeasbilityStatus == feasable)
				{
					//add the last sample point
					Node* newNode = insertNode(*parentNode, goalSamplePair.first, *goalTrajectoryListOut.back());
					if (newNode == NULL)
						return 0;

					//update lowerbound cost-to-go
					State state(goalPoint->wp.x, goalPoint->wp.y, 0, 0, goalPoint->heading, 0, 0);
					newNode->lowerBoundCost =
						simulationModel->evaluateEuclidianCostToGo(newNode->getState(), state);

					//update the upperbound cost-to-go
					newNode->upperBoundCost = goalTrajectoryListOut.back()->cost;
					updateUpperBoundCost(*newNode);

					//add node to safe stopping set and mark intermediate nodes as safe
					stoppingNodeList.push_front(newNode);
					propagateSafeState(*newNode);

					//plot info
					newNode->isSampleNode = true;
				}

				return 1;
			}
		}
	}
	
	return 0;
}

void    Planner::resetTree()
{
	Node* rootBackup = NULL;
	if (root)
	{
		rootBackup = new Node(*root);
		rootBackup->children.clear();
	}

	// Delete all the vertices
	for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
		delete *iter;

	// Delete all the vertices
	for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
		delete *iter;

	forwardNodeList.clear();
	stoppingNodeList.clear();
	reverseNodeList.clear();
	lastBestSequence.clear();
	nextLastBestSequence.clear();

	numForwardNodes = 0;
	numReverseNodes = 0;
	numNodes = 0;
	lowerBoundCost = DBL_MAX;
	lowerBoundNode = NULL;
	nextLowerBoundNode = NULL;
	nextLowerBoundCost = DBL_MAX;

	if (rootBackup->getDirection() == forwardDirection)
	{
		forwardNodeList.push_back(rootBackup);
		numForwardNodes++;
		numNodes++;
	}
	else
	{
		reverseNodeList.push_back(rootBackup);
		numReverseNodes++;
		numNodes++;
	}

	root = rootBackup;
}

void    Planner::removeOldNodes()
{
	if (lastBestSequence.empty())
		return;

	//This must be shared as a parametera!!! //TODO
	int lookAheadDistance = 10;

	/****************************************************************/
	/*** Setup varibles   *******************************************/
	/****************************************************************/
	bool nextWpEventIndicator = false;

	/****************************************************************/
	/*** Get current way point  *************************************/
	/****************************************************************/
	if (nextLowerBoundNode != NULL)
	{
		if (nextLastBestSequence.empty())
			return;

		Node* currentControlNode = nextLastBestSequence.front();

		while (sqrt((currentControlNode->getStateKey().x - egoState->x)*(currentControlNode->getStateKey().x - egoState->x) +
			(currentControlNode->getStateKey().y - egoState->y)*(currentControlNode->getStateKey().y - egoState->y)) <= lookAheadDistance)
		{

			//indicate a new control point if it is not first entry
			nextWpEventIndicator = true;

			//remove old point and set it as lastWp
			nextLastBestSequence.pop_front();

			/****************************************************************/
			/*** Check so the reference is not empty   **********************/
			/****************************************************************/
			if (nextLastBestSequence.empty())
			{
				return;
			}

			//set new control
			currentControlNode = nextLastBestSequence.front();
		}

		if (nextWpEventIndicator)
		{
			Node* rootCandidate = currentControlNode;
			setNewRootNode(*rootCandidate);
		}
	}
	else
	{
		Node* currentControlNode = lastBestSequence.front();

		while (sqrt((currentControlNode->getStateKey().x - egoState->x)*(currentControlNode->getStateKey().x - egoState->x) +
			(currentControlNode->getStateKey().y - egoState->y)*(currentControlNode->getStateKey().y - egoState->y)) <= lookAheadDistance)
		{

			//indicate a new control point if it is not first entry
			nextWpEventIndicator = true;

			//remove old point and set it as lastWp
			lastBestSequence.pop_front();

			/****************************************************************/
			/*** Check so the reference is not empty   **********************/
			/****************************************************************/
			if (lastBestSequence.empty())
			{
				return;
			}

			//set new control
			currentControlNode = lastBestSequence.front();
		}

		if (nextWpEventIndicator)
		{
			Node* rootCandidate = currentControlNode;
			setNewRootNode(*rootCandidate);
		}

	}
}

/****************************************************************/
/*** Main initzialisation  **************************************/
/****************************************************************/
void  Planner::initialize()
{

	// If there is no system, then return failure
	if (!simulationModel)
		printf("Can not initialize without a model! Use setModel first!");

	// Backup the root
	Node* rootBackup = NULL;
	if (root)
		rootBackup = new Node(*root);

	// Delete all the vertices
	for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
		delete *iter;

	// Delete all the vertices
	for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
		delete *iter;

	forwardNodeList.clear();
	stoppingNodeList.clear();
	reverseNodeList.clear();

	numForwardNodes = 0;
	numReverseNodes = 0;
	numNodes = 0;
	lowerBoundCost = DBL_MAX;
	lowerBoundNode = NULL;
	nextLowerBoundNode = NULL;
	nextLowerBoundCost = DBL_MAX;


	root = rootBackup;
	if (root)
	{
		root->children.clear();
		forwardNodeList.push_back(root);
		stoppingNodeList.push_back(root);
		numForwardNodes++;
		numNodes++;
	}

	prevGoalPointID = goalPoint->ID;

}

/****************************************************************/
/*** Main intteration  ******************************************/
/****************************************************************/
bool Planner::itterate(Model& simulationModel)
{
	/****************************************************************/
	/*** Check if global goal is reached  ***************************/
	/****************************************************************/
	if (!checkGlobalGoal())
		return false;

	/****************************************************************/
	/*** Set new root if the waypoint has been taken  ***************/
	/****************************************************************/
	removeOldNodes();

	return true;
}

int Planner::expandTree()
{

	/****************************************************************/
	/*** Sample state  **********************************************/
	/****************************************************************/
	pair<Sample, Sample> randomSample;
	pair<Sample, Sample> goalSample;


	switch (behaviourState)
	{
	case Behaviour::Safety_Behaviour:
		samplingStategies->Safety_mode_sampling(randomSample, goalSample, *egoState, *goalPoint, *nextGoalPoint);
		break;
	case Behaviour::Racing_Behaviour:
		samplingStategies->Racing_mode_sampling(randomSample, goalSample, *egoState, *goalPoint, *nextGoalPoint, fesibleTrajectoryFound());
		break;
	default:
		break;
	}
	randomSample.first.velocity = goalPoint->velocity;
	goalSample.first.velocity = goalPoint->velocity;

	/****************************************************************/
	/*** Extend tree towards sample  ********************************/
	/****************************************************************/
	extendTo(randomSample, goalSample);


	return 0;

}

bool Planner::fesibleTrajectoryFound()
{
	bool fessible = false;

	if (lowerBoundNode != NULL || nextLowerBoundNode != NULL)
		fessible = true;


	return fessible;
}


Node& Planner::getBestNode()
{
	if (nextLowerBoundNode)
		return *nextLowerBoundNode;
	else
		return *lowerBoundNode;
}