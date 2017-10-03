#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <set>


#include "Model.h"
#include "MapHandler.h"
#include "SamplingStrategies.h"
#include "Planner.h"

using namespace std;

class ReactivePlanner : public Planner
{

    /****************************************************************/
    /*** Private varibles *******************************************/
    /****************************************************************/
    int maxConnectionTries;

    //pointer to a simulation model used for forward propagation
    Model       *simulationModel;

    //pointer to an obstacleMap used for collision checks
    MapHandler  *mapHandler;



    /****************************************************************/
    /*** Setup of random number generator ***************************/
    /****************************************************************/
    default_random_engine generator;
    uniform_real_distribution<double> stdUniformDist;

    SamplingStrategies  *samplingStategies;

    /****************************************************************/
    /*** Private methods ********************************************/
    /****************************************************************/

    /****************************************************************/
    /*** Tree construction methods **********************************/
    /****************************************************************/
    int extendTo(Sample sample);



    /****************************************************************/
    /*** Misc                      **********************************/
    /****************************************************************/
    void debugPrintToFile();

public:

    /****************************************************************/
    /*** Constructor destructor *************************************/
    /****************************************************************/
    ReactivePlanner ();
    ~ReactivePlanner ();

    /****************************************************************/
    /*** Set methods ************************************************/
    /****************************************************************/
    void setModel (Model* simulationModelIn);
    void setMapHandler(MapHandler* mapHandlerIn);
    void setEgoState(State* stateIn);
    void setRoadCenter(vector<Point2D> roadCenter_in);

    /****************************************************************/
    /*** Get methods ************************************************/
    /****************************************************************/
    int                     getBestControl (list<State>& controlOut);
    MapHandler&             getMapHandler(){return *mapHandler;}
    Behaviour               getBehaviourState(){return behaviourState;}

    /****************************************************************/
    /*** Tree construction methods **********************************/
    /****************************************************************/
    pathFeasability         simulateToNode(Node& nodeFromIn, Sample& sample, Trajectory& trajectoryOut, MapHandler& mapHandler, double Tsim);
    pathFeasability         simulatePath(Node& nodeFromIn, double offset, Trajectory& trajectoryOut, MapHandler& mapHandler, double Tsim);

    /****************************************************************/
    /*** Initzialisation of the tree ********************************/
    /****************************************************************/
    void initialize();

    /****************************************************************/
    /*** Main itteration function  **********************************/
    /****************************************************************/

    //extends samples to the tree
    int expandTree();
    int updateReferencePath();

    int doPlanningCycle(list<State>& bestControlTrajectory);

};
