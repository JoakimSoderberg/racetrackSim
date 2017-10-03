#pragma once
#define _USE_MATH_DEFINES

#include <iostream>
#include <vector>

#include "PlannerTypes.h"
#include "Geometry.h"
#include "ModelParameters.h"

typedef struct
{
    double accelerationCommand;
    double steerAngleCommand;
} ControlCommand;

/****************************************************************/
/*** Controller class: Add a controller to a model class to  ****/
/*** calculate control inputs   *********************************/
/****************************************************************/
class PurePursuitController
{
    /****************************************************************/
    /*** Private varibles ********************************************/
    /****************************************************************/

    private:
        //keep track of last visited point
        Point2D lastWp;

        //keep track of last added intermediate point
        Point2D lastIndermediatePoint;

        //used to give outside classes indication of a new control point
        bool nextWpEventIndicator;
        bool isOkAddNewIntermediateNodeIndicator;
        bool wpInsideLookAheadIndicator;

        State       *egoState;
        Parameters  *egoParameters;

    public:

        //list to hold the reference control path
        //list<Node*>	referenceControl;
        list<pair<Point2D, double> >   referenceControl;

        //varibles to hold the current target control point
        Point2D 	currentWayPoint;
        double      currentVref;
        Node*		currentControlNode;

        //varible to hold the the look ahead points on the look ahead circle
        vector<Point2D> lookAheadPoints;

        //parameter setting the look ahead distance
        double lookAheadDistance;

        /****************************************************************/
        /*** Constructor/Destructor  ************************************/
        /****************************************************************/
        PurePursuitController();

        /****************************************************************/
        /*** Public methods *********************************************/
        /****************************************************************/
        void    initzialise();
        int     handleWayPoints();
        int		calcControl(ControlCommand &c, double &vRef);
        //void	setReferenceControl(list<Node*> referenceControl_in);
        void	setReferenceControl(list<pair<Point2D, double> > referenceControl_in);
        bool	nextControlWpEvent(){return nextWpEventIndicator;};
        bool	wpInsideLookAhead(){return wpInsideLookAheadIndicator;};
        bool	isOkAddNewIntermediateNode(){return isOkAddNewIntermediateNodeIndicator;};
        Point2D getLastWp(){return lastWp;};

        void setState(State* egoState_in)
        {
            egoState = egoState_in;
        }

        void setParameters(Parameters* egoParameters_in)
        {
            egoParameters = egoParameters_in;
        }

};
