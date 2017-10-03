#pragma once

#include "Planner.h"
#include "LCMHandler.h"
#include "MapHandler.h"

#ifdef UBUNTU
class BTHandler
{
public:



    BTHandler(){}
    ~BTHandler(){}

    void setPlanner(Planner* planner_in){planner = planner_in;}
    void setLCMHandler(LCMHandler* lcmHandler_in){lcmHandler = lcmHandler_in;}
    void setMapHandler(MapHandler* mapHandler_in){mapHandler = mapHandler_in;}

    void updateBehaviourTree();


private:
    Planner     *planner;
    LCMHandler  *lcmHandler;
    MapHandler  *mapHandler;
};
#endif
