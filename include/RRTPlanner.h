#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <set>


#include "Model.h"
#include "MapHandler.h"
#include "SamplingStrategies.h"

using namespace std;

class Planner 
{

    /****************************************************************/
    /*** Private varibles *******************************************/
    /****************************************************************/
    int maxConnectionTries;

    //Pointer to the current best end node
    double lowerBoundCost;
    Node* lowerBoundNode;

    double nextLowerBoundCost;
    Node* nextLowerBoundNode;

    //Pointer to the root of the tree
    Node* root;

    //Pointer to the current goal and previous
    WayPoint* goalPoint;

    WayPoint previousGoalPoint;

	WayPoint* nextGoalPoint;

    //used to keep track of when the current goal has changed
    int prevGoalPointID;

    //global goal for the robot
    list<WayPoint> globalReferencePath;

	//golbal nextgoal for the robot
	list<WayPoint> globalNextReferencePath;

    //keep track of visited waypoints(just for plotting)
    list<WayPoint> visitedWayPoints;

    //list of last low cost sequence
    list<Node*> lastBestSequence;

	//list of last now cost sequence to next goal
	list<Node*> nextLastBestSequence;

    //pointer to a simulation model used for forward propagation
    Model *simulationModel;

    //pointer to an obstacleMap used for collision checks
    MapHandler* mapHandler;

    //pointer to the current state of the vehicle
    State *egoState;

    //Behaviour state
    Behaviour behaviourState;

    /****************************************************************/
    /*** Setup of random number generator ***************************/
    /****************************************************************/
    default_random_engine generator;
    uniform_real_distribution<double> stdUniformDist;

    SamplingStrategies* samplingStategies;

    /****************************************************************/
    /*** Private methods ********************************************/
    /****************************************************************/
    
    /****************************************************************/
    /*** Tree construction methods **********************************/
    /****************************************************************/
    Node*                   insertNode(Node& nodeStartIn,  Sample &randomSample, Trajectory& trajectoryIn);
    int						insertNode(Node& vertexStartIn, Trajectory& trajectoryIn, Node& vertexEndIn);
    void					insertIntermediateNodes(Node*& parentNode, list<Point2D>& intermediateControlPoints, list<Trajectory*>	trajectoryListOut, list<Node*> addedNodesOut, Direction direction, double vRef);
    int						deleteChild(Node& nodeIn, Node& keepNode);
    pathFeasability			seekTreeConnection(pair<Sample, Sample>& samplePair, Node*& bestNode, list<Trajectory*>& trajectoryListOut, list<Point2D>& intermediateControlPoints);
    pathFeasability			seekGoalConnection(pair<Sample, Sample>& goalSamplePair, Node& startNode, list<Trajectory*>& trajectoryListOut, list<Point2D>& intermediateControlPoints);
    void                    removeOldNodes();
    int						extendTo(pair<Sample, Sample> sampleTo, pair<Sample, Sample> goalSamplePair);

    /****************************************************************/
    /*** Misc                      **********************************/
    /****************************************************************/
    int         checkUpdateBestNode(Node& nodeIn);
    void        recalculateCostToGo();
    void        updateUpperBoundCost(Node& nodeIn);
    void        calculateHeuristicCost(Sample &sampleIn, list<Node*> &nodeList, vector<pair<Node*, double>> &nodeCostPairVector, bool optimization);
    void        calculateHeuristicCostToStoppingNodes(Sample &sampleIn, list<Node*> &nodeList, vector<pair<Node*, double>> &nodeCostPairVector, bool optimization);
    void        propagateSafeState(Node& nodeIn);
	void		markForDeletion(Node& keepFromNode);

	//Functions that are used to check if (next)LowerBoundNode is fessible.
	void		lazy_check();
	bool		check_lower_bound_node();
	bool		check_next_lower_bound_node();




    void		debugPrintToFile();

public:

    /****************************************************************/
    /*** Public varibles ********************************************/
    /****************************************************************/

    //list to store the tree
    list<Node*> forwardNodeList;
    list<Node*> reverseNodeList;
    list<Node*> stoppingNodeList;

    int numNodes;
    int numForwardNodes;
    int numReverseNodes;

    /****************************************************************/
    /*** Constructor destructor *************************************/
    /****************************************************************/
    Planner (double Ts_in);
    ~Planner ();

    /****************************************************************/
    /*** Set methods ************************************************/
    /****************************************************************/
    void setModel (Model* simulationModelIn);
    void setMapHandler(MapHandler* mapHandlerIn);
    void setEgoState(State* stateIn);
    void setGoal(WayPoint* goalPointIn);
	void setNextGoal(WayPoint* nextGoalPointIn);
    void setNewRootNode(Node& newRoot);
    void setGlobalReferencePath(list<WayPoint> globalReference_in,list<WayPoint> globalNextReference_in);
    void setRoadCenter(vector<Point2D> roadCenter_in);
	

    /****************************************************************/
    /*** Get methods ************************************************/
    /****************************************************************/
    int                 getBestControl (list<pair<Point2D, double>>& controlOut);
	Node&               getBestNode();
    Node&               getRootNode ();
    MapHandler&         getMapHandler(){return *mapHandler;}
    list<WayPoint>&     getGlobalReferencePath(){return globalReferencePath;}
	list<WayPoint>&		getGlobalNextReferencePath(){return globalNextReferencePath;}
    list<WayPoint>&     getVisitedWayPoints(){return visitedWayPoints;}
    Behaviour           getBehaviourState(){return behaviourState;}
	

    /****************************************************************/
    /*** Tree construction methods **********************************/
    /****************************************************************/
    int             deleteBranch(Node& nodeIn, Node& keepNode);
    void            resetTree();

    bool            checkGlobalGoal();

    /****************************************************************/
    /*** Initzialisation of the tree ********************************/
    /****************************************************************/
    void initialize();

    /****************************************************************/
    /*** Main itteration function  **********************************/
    /****************************************************************/
    
	list<ReferenceTrajectoryState> calculate_reference_trajectory(double Ts);

	bool		fesibleTrajectoryFound();

    //call once before expand loop
    bool itterate(Model& simulationModel);

    //extends samples to the tree
    int expandTree();

	//time between states in a node
	double Ts;

	//calculates time from start_node to root.
	double time_from_node_to_root(Node* start_node);

};
