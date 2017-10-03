#include "PurePursuitController.h"

/****************************************************************/
/*** Controller class: Add a controller to a model class to  ****/
/*** calculate control inputs   *********************************/
/****************************************************************/

/****************************************************************/
/*** Constructor/Destructor  ************************************/
/****************************************************************/
PurePursuitController::PurePursuitController()
{
    lastWp = Point2D();

    //TODO: Share as a parameter
    lookAheadDistance = 10;		//changed from 10.
    lookAheadPoints.push_back(Point2D());
    lookAheadPoints.push_back(Point2D());
    currentWayPoint = Point2D();
}

/****************************************************************/
/*** Public methods *********************************************/
/****************************************************************/
//void	PurePursuitController::setReferenceControl(list<Node*> referenceControl_in)
void	PurePursuitController::setReferenceControl(list<pair<Point2D, double>> referenceControl_in)
{
    referenceControl = referenceControl_in;
    //currentControlNode = referenceControl.front();
}

int    PurePursuitController::handleWayPoints()
{
    /****************************************************************/
    /*** Setup varibles   *******************************************/
    /****************************************************************/
    nextWpEventIndicator = false;
    wpInsideLookAheadIndicator  = false;

    /****************************************************************/
    /*** Check so the reference is not empty   **********************/
    /****************************************************************/
    //If no more waypoints give zero command (in future maybe emergency break!!)

	

    if(referenceControl.empty())
    {
        egoState->v = 0;
        lastWp = Point2D(0,0);
        return 0;
    }

    /****************************************************************/
    /****************************************************************/
    /*** WAYPOINT HANDLING   ****************************************/
    /****************************************************************/
    /****************************************************************/

    /****************************************************************/
    /*** Get current way point  *************************************/
    /****************************************************************/
    
    currentWayPoint = referenceControl.front().first;
    currentVref     = referenceControl.front().second;

    /****************************************************************/
    /*** check if control point is reached   ************************/
    /****************************************************************/
	
    while(sqrt((currentWayPoint.x - egoState->x)*(currentWayPoint.x - egoState->x) + (currentWayPoint.y - egoState->y)*(currentWayPoint.y - egoState->y))  <= lookAheadDistance)
    {
        /****************************************************************/
        /*** Set varibles for next waypoint   ***************************/
        /****************************************************************/

        //indicate a new control point if it is not first entry
        nextWpEventIndicator = true;

        //remove old point and set it as lastWp
        referenceControl.pop_front();
        lastWp = currentWayPoint;

        //init of lastIndermediatePoint. Used for determining when it is time to add a intermediate node
        lastIndermediatePoint = lastWp;

        /****************************************************************/
        /*** Check so the reference is not empty   **********************/
        /****************************************************************/
        //If no more waypoints give zero command (in future maybe emergency break!!)
        if(referenceControl.empty())
        {
            egoState->v = 0;
            lastWp = Point2D(0,0);

            return 1;
        }


        /****************************************************************/
        /*** Set new way point ******************************************/
        /****************************************************************/

        currentWayPoint = referenceControl.front().first;
        currentVref     = referenceControl.front().second;

    }

    return -99;
}

int	PurePursuitController::calcControl(ControlCommand &c, double &vRef)
{
    isOkAddNewIntermediateNodeIndicator = false;

    int returnValue = handleWayPoints();

    //-99 means continue, 0 no more waypoints, 1 no more waypoints: Make this more readalbe!!!
    if(returnValue != -99)
    {
        c.accelerationCommand = 0;
        c.steerAngleCommand = 0;
        vRef = 0;

        return returnValue;
    }

    /****************************************************************/
    /****************************************************************/
    /*** CONTROL CALCULATION  ***************************************/
    /****************************************************************/
    /****************************************************************/

    /****************************************************************/
    /*** Setup for varibles  ****************************************/
    /****************************************************************/
    Point2D carLocation;
    carLocation.x = egoState->x;
    carLocation.y = egoState->y;

    //calculate circle line intersection and inducate circle lost if no intersection is found. Current wayPoints is returned if circle is lost.
    //in lookAheadPoints first element is always close st to next waypoint and second is closest to last waypoint
    int circleLost = !getLineCircleIntersections(lastWp, currentWayPoint, carLocation, lookAheadDistance, lookAheadPoints);

    if(circleLost)
        return 0;


    /****************************************************************/
    /*** Logic to check if it is ok to add a new intermediate node **/
    /****************************************************************/
    if(isOkAddIntermediateWayPoint(lookAheadPoints[0], lastIndermediatePoint, currentWayPoint))
    {
        lastIndermediatePoint = lookAheadPoints[0];
        isOkAddNewIntermediateNodeIndicator = true;
    }


    /****************************************************************/
    /*** Calculate control input  ***********************************/
    /****************************************************************/
    //calculate steer control
    double lookAheadHeading = atan2(lookAheadPoints[0].y - egoState->y, lookAheadPoints[0].x - egoState->x);


    /****************************************************************/
    /*** Hack to fix problem when waypoint is right behind vehicle **/
    /****************************************************************/
    if(lookAheadHeading < -(M_PI-0.01))
    {
        lookAheadHeading -= 0.1;

    }
    else if(lookAheadHeading > (M_PI-0.01))
    {
        lookAheadHeading -= 0.1;
    }
	
    //calculate the error heading
    double errorHeading = egoState->heading - lookAheadHeading;

	double Gss = 1 / (1 + abs(egoState->v) / 20);

	//Gss = 1;
    //calculate steer angle command according to non-linear control law
	c.steerAngleCommand = -atan(2 * egoParameters->L * sin(errorHeading) / (Gss*lookAheadDistance));

    vRef = currentVref;

    return 1;

}

void PurePursuitController::initzialise()
{
        handleWayPoints();
}
