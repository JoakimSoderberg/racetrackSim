#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <set>


#include "TreeStructure.h"
#include "Model.h"

using namespace std;

class Planner
{

public:

    TreeStructure   planningTree;
    string          plannerType;

    //Behaviour state
    Behaviour       behaviourState;

    //pointer to the current state of the vehicle
    State           *egoState;

    //reference point at center of road
    vector<Point2D>                 tempRoadCenter;

    PathGenerator   pathGenerator;

    Planner(){}
    ~Planner(){}

    Behaviour                           getBehaviourState(){return behaviourState;}
    //scania_lcm::tSITAW_WAYPOINT_STR&    getRoadCenter(){return roadCenter;}

    bool onRoadStatus;
    bool idleStatus;

    /****************************************************************/
    /*** VIRTUAL METHODS ********************************************/
    /****************************************************************/
    /****************************************************************/
    /*** Set methods ************************************************/
    /****************************************************************/
    virtual void    setGlobalReferencePath(list<WayPoint> globalReference_in){}
    virtual void    setModel (Model* simulationModelIn){}
    virtual void    setMapHandler(MapHandler* mapHandlerIn){}
    virtual void    setEgoState(State* stateIn){}
    virtual void    setGoal(WayPoint* goalPointIn){}
    virtual void    setRoadCenter(vector<Point2D> roadCenter_in){}
    virtual void    setOnRoadStatus(){}
    virtual void    setOpenAreaStatus(){}
    virtual void    setIdle(){}

    /****************************************************************/
    /*** Get methods ************************************************/
    /****************************************************************/
	
	list<WayPoint> temp;

	virtual list<WayPoint>&     getGlobalReferencePath()
	{ 
		return temp; 
	}
    virtual list<WayPoint>&     getVisitedWayPoints()
	{
		return temp;
	}
    /****************************************************************/
    /*** Misc *******************************************************/
    /****************************************************************/
	virtual bool    isIdle(){ return 0; }

    /****************************************************************/
    /*** Initzialisation of the tree ********************************/
    /****************************************************************/
    virtual void    initialize(){}

    /****************************************************************/
    /*** Main call to planner ***************************************/
    /****************************************************************/
	virtual int     doPlanningCycle(list<State>& bestControlTrajectory){ return 0; }

};
