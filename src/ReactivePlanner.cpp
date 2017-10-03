#include <algorithm>    // std::sort
#include "ReactivePlanner.h"

/****************************************************************/
/*** Constructor destructor *************************************/
/****************************************************************/
ReactivePlanner::ReactivePlanner ()
{
    plannerType = "ReactivePlanner";

    samplingStategies = new SamplingStrategies();

    maxConnectionTries = 10;

    simulationModel = NULL;
}

ReactivePlanner::~ReactivePlanner ()
{

}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void	ReactivePlanner::setModel (Model* simulationModelIn)
{

    if (simulationModel)
        delete simulationModel;

    simulationModel = simulationModelIn;

}

void	ReactivePlanner::setEgoState(State* const stateIn)
{
    egoState = stateIn;
}

void	ReactivePlanner::setMapHandler(MapHandler* mapHandlerIn)
{
    mapHandler = mapHandlerIn;
}

void    ReactivePlanner::setRoadCenter(vector<Point2D> roadCenter_in)
{
    //roadCenter = roadCenter_in;
   // pathGenerator.generateNormals(roadCenter);
}

/****************************************************************/
/*** Get methods ************************************************/
/****************************************************************/
int		ReactivePlanner::getBestControl  (list<State>& controlOut)
{
   return planningTree.getLowCostTrajectory(controlOut);
}


/****************************************************************/
/*** Tree construction methods **********************************/
/****************************************************************/

pathFeasability		ReactivePlanner::simulateToNode(Node& nodeFromIn, Sample& sample, Trajectory& trajectoryOut, MapHandler& mapHandler, double Tsim)
{

    //MOVE THIS TO SOME PARAMETER INGS!!
    double Ts = 0.04;

    /****************************************************************/
    /*** Simulation setup *******************************************/
    /****************************************************************/

    //set simulation start state to start state
    simulationModel->state  =  nodeFromIn.getState();
    simulationModel->state.vx  =  sample.velocity;


    //create a local reference control with only start and end point
    list<State> referenceControl_in;

    //add start way point
    State temp;

    temp.x      = nodeFromIn.getStateKey().x;
    temp.y      = nodeFromIn.getStateKey().y;
    temp.vx      = 0;
    referenceControl_in.push_back(temp);

    //add end way point
    temp.x      = sample.samplePoint.x;
    temp.y      = sample.samplePoint.y;
    temp.vx      = 0;
    referenceControl_in.push_back(temp);

    State temp2;
    sample.samplePoint.x = sample.samplePoint.x + 200;
    temp2.x     = sample.samplePoint.x;
    temp2.y     = sample.samplePoint.y;
    temp2.vx     = 0;
    referenceControl_in.push_back(temp2);

    //add list to controller
    simulationModel->controller->setReferenceControl(referenceControl_in);


    /****************************************************************/
    /*** Simulation loop ********************************************/
    /****************************************************************/
    double simulationTime   = 0;
    int    numberOfSteps    = 0;
    ControlCommand c;
    double vRef;

    while(!simulationModel->controller->referenceControl.empty() && simulationTime <= Tsim)
    {
        numberOfSteps++;

        /****************************************************************/
        /*** Calculate steer angle control ******************************/
        /****************************************************************/
        if(!simulationModel->controller->calcControl(c, vRef))
        {
            return nonFeasable;
        }

        /****************************************************************/
        /*** Simulate system with the control input *********************/
        /****************************************************************/
        simulationModel->step(Ts, c.steerAngleCommand, 0);

        /****************************************************************/
        /*** Perform collision checks with the map **********************/
        /**************************i**************************************/

//        if(mapHandler.getCost(simulationModel->state, simulationModel->parameters) > 1 )
//        {
//            if(trajectoryOut.stateList.empty())
//                return nonFeasable;
//            else
//            {
//                return partlyFeasable;
//            }
//        }

        /****************************************************************/
        /*** Update cost and add trajectory states **********************/
        /****************************************************************/
        trajectoryOut.cost += sqrt((simulationModel->state.x - trajectoryOut.stateList.back().x)*(simulationModel->state.x - trajectoryOut.stateList.back().x) + (simulationModel->state.y - trajectoryOut.stateList.back().y)*(simulationModel->state.y - trajectoryOut.stateList.back().y));
        trajectoryOut.stateList.push_back(simulationModel->state);

        simulationTime += Ts;

    }


    /****************************************************************/
    /*** If program reached here everything went well ***************/
    /****************************************************************/

    return feasable;
}

pathFeasability		ReactivePlanner::simulatePath(Node& nodeFromIn, double offset, Trajectory& trajectoryOut, MapHandler& mapHandler, double Tsim)
{

    //MOVE THIS TO SOME PARAMETER INGS!!
    double Ts = 0.1;

    /****************************************************************/
    /*** Simulation setup *******************************************/
    /****************************************************************/

    //set simulation start state to start state
    simulationModel->state      =  nodeFromIn.getState();
    simulationModel->state.vx    =  10;


    //create a local reference control with only start and end point
    list<State> referenceControl_in;

    pathGenerator.generateOffsetPathList(offset, referenceControl_in);

    //add start way point
    State temp;
    temp.x  = nodeFromIn.getStateKey().x;
    temp.y  = nodeFromIn.getStateKey().y;
    temp.vx  = 0;
    referenceControl_in.push_front(temp);


    //add list to controller
    simulationModel->controller->setReferenceControl(referenceControl_in);


    /****************************************************************/
    /*** Simulation loop ********************************************/
    /****************************************************************/
    double simulationTime   = 0;
    int    numberOfSteps    = 0;
    ControlCommand c;
    double vRef;

    while(!simulationModel->controller->referenceControl.empty() && simulationTime <= Tsim)
    {
        numberOfSteps++;

        /****************************************************************/
        /*** Calculate steer angle control ******************************/
        /****************************************************************/
        if(!simulationModel->controller->calcControl(c, vRef))
        {
            return nonFeasable;
        }

        /****************************************************************/
        /*** Simulate system with the control input *********************/
        /****************************************************************/
        simulationModel->step(Ts, c.steerAngleCommand, 0);

        /****************************************************************/
        /*** Perform collision checks with the map **********************/
        /****************************************************************/

        if(mapHandler.getCost(simulationModel->state, simulationModel->parameters) > 1 )
        {
            if(trajectoryOut.stateList.empty())
            {
                return nonFeasable;
            }
            else
            {
                return partlyFeasable;
            }
        }

        /****************************************************************/
        /*** Update cost and add trajectory states **********************/
        /****************************************************************/
        //trajectoryOut.cost -= sqrt((simulationModel->state.x - trajectoryOut.stateList.back().x)*(simulationModel->state.x - trajectoryOut.stateList.back().x) + (simulationModel->state.y - trajectoryOut.stateList.back().y)*(simulationModel->state.y - trajectoryOut.stateList.back().y));
        //trajectoryOut.cost -= sqrt((simulationModel->state.x - tempRoadCenter[0].x)*(simulationModel->state.x - tempRoadCenter[0].x) + (simulationModel->state.y - tempRoadCenter[0].y)*(simulationModel->state.y - tempRoadCenter[0].y));
        trajectoryOut.cost += mapHandler.getCenterDistance(simulationModel->state);

        trajectoryOut.stateList.push_back(simulationModel->state);

        simulationTime += Ts;

    }


    /****************************************************************/
    /*** If program reached here everything went well ***************/
    /****************************************************************/

    return feasable;
}

int		ReactivePlanner::extendTo(Sample sample)
{

    //Function varibles
    Trajectory simulatedTrajectory;
    pathFeasability sampleFeasbilityStatus = nonFeasable;


    Node* parentNode = &planningTree.getRootNode();

    /****************************************************************/
    /*** Find best extentions in tree *******************************/
    /****************************************************************/

    sampleFeasbilityStatus = simulatePath(*parentNode, sample.velocity, simulatedTrajectory, *mapHandler, 10);
    //sampleFeasbilityStatus = simulateToNode(*parentNode, sample, simulatedTrajectory, *mapHandler, 10);

    //add all nodes if path is feasable and only feasable part if path is partly feasable
    if(sampleFeasbilityStatus == feasable || sampleFeasbilityStatus == partlyFeasable)
    {

        /****************************************************************/
        /*** Add new node and intermediate nodes to the tree  ***********/
        /****************************************************************/

        //only add sample node if the path is fully feasable
        if(sampleFeasbilityStatus == feasable)
        {
            //add the last sample point
            Node* newNode = planningTree.insertNode (*parentNode, sample, simulatedTrajectory);
            if (newNode == NULL)
                return 0;

            //plot info
            newNode->isSampleNode = true;

            //add node to safe stopping set and mark intermediate nodes as safe
            planningTree.stoppingNodeQueue.push(newNode);
            planningTree.stoppingNodeList.push_front(newNode);
            planningTree.propagateSafeState(*newNode);
        }


    }
}

/****************************************************************/
/*** Main initzialisation  **************************************/
/****************************************************************/
void  ReactivePlanner::initialize()
{

    // If there is no system, then return failure
    if (!simulationModel)
        printf("Can not initialize without a model! Use setModel first!");

    planningTree.setRootState(simulationModel->state, simulationModel->controller->lookAheadDistance);
    planningTree.initialize();

}

/****************************************************************/
/*** Main intteration  ******************************************/
/****************************************************************/
int ReactivePlanner::expandTree()
{

    /****************************************************************/
    /*** Sample state  **********************************************/
    /****************************************************************/
    Sample randomSample;

//    switch (behaviourState)
//    {
//    case Behaviour::normalParking:
//        samplingStategies->reactiveSample(randomSample, *egoState);
//        //randomSample.first.samplePoint = samplingStategies->drawUniformOnRectangle(0,0,100,100);
//        break;
//    case Behaviour::forwardOnRoad:
//        samplingStategies->reactiveSample(randomSample, *egoState);
//        break;
//    case Behaviour::leaveParking:
//       samplingStategies->reactiveSample(randomSample, *egoState);
//        break;
//    case Behaviour::reverseParking:
//       samplingStategies->reactiveSample(randomSample, *egoState);
//        break;
//    case Behaviour::Uturn:
//      samplingStategies->reactiveSample(randomSample, *egoState);
//        break;

//    default: ;
//        break;
//    }
    /****************************************************************/
    /*** Extend tree towards sample  ********************************/
    /****************************************************************/
    samplingStategies->reactiveSample(randomSample, *egoState);
    samplingStategies->test(randomSample, *egoState);
//    for(int i = -10; i < 10; i++)
//    {
        randomSample.velocity = randomSample.samplePoint.y;
        extendTo(randomSample);
//    }
    return 0;

}

int ReactivePlanner::updateReferencePath()
{
    vector<Point2D> temp;

    tempRoadCenter.clear();
    double lookAheadDistance = 10;
    Point2D p1 = Point2D(lookAheadDistance*cos(egoState->heading) + egoState->x   , lookAheadDistance*sin(egoState->heading) + egoState->y);
    Point2D p2 = Point2D(lookAheadDistance*cos(egoState->heading + M_PI/2) + p1.x , lookAheadDistance*sin(egoState->heading  + M_PI/2) + p1.y);


    static int i_start = 1;
    //while(sqrt((roadCenter[i_start].x - egoState->x)*(roadCenter[i_start].x - egoState->x) +  (roadCenter[i_start].y - egoState->y)*(roadCenter[i_start].y - egoState->y))  <= lookAheadDistance)
    //while(!sideOfLine(p1, p2, roadCenter[i_start]))
    {
        i_start++;
    }

//    if(i_start != 0)
//        int stoop = 3;

  //  for(int i = i_start-1; i<roadCenter.size(); i++)
    {
    //    tempRoadCenter.push_back(roadCenter[i]);
    }

    pathGenerator.generateNormals(tempRoadCenter);
	return 0; //OL
}

int ReactivePlanner::doPlanningCycle(list<State>& bestControlTrajectory)
{
        int numberOfItterations = 0;

       // clock_t start = clock(); //OL
        double deltaT = 0.2;

        planningTree.resetTree();
        planningTree.setRootState(*egoState, 9);
        updateReferencePath();

        /*while((float)((clock() - start))/CLOCKS_PER_SEC < deltaT)
        {
            expandTree();
            numberOfItterations++;
        }*/ //OL

        //printf("Time in iteration: %f Num iterations: %d\n", (float)((clock() - start))/CLOCKS_PER_SEC, numberOfItterations);

//        pathGenerator.generateOffsetPathList(0, bestControlTrajectory);

//        //add start way point
//        pair<Point2D, double> temp;
//        temp.first.x = egoState->x;
//        temp.first.y = egoState->y;
//        temp.second    = 0;
//        bestControlTrajectory.push_front(temp);
        return getBestControl(bestControlTrajectory);
}
