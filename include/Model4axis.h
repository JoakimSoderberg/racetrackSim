#pragma once
//#define _USE_MATH_DEFINES

//#include <iostream>
//#include <vector>
//#include <queue>
//#include <list>
//#include <random>

#include "Model.h"

#ifdef UBUNTU
	#include "trin_feedback2.h"
	#include "axis4Model.h"
#endif

//#include "ModelParameters.h"
//#include "Geometry.h"
//#include "PlannerTypes.h"
//#include "MapHandler.h"


using namespace std;


/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/
class Model4axis : public Model
{
    private:

    public:

        /****************************************************************/
        /*** Constructor Destructor  ************************************/
        /****************************************************************/
        Model4axis(int ID);
        Model4axis(int ID, double lookAheadDistance);
        ~Model4axis();

        /****************************************************************/
        /*** Set methods ************************************************/
        /****************************************************************/
        void set_state(double x_in, double y_in, double vx_in, double vy_in, double a_in, double heading_in, double yawRate_in, double steerAngle_in, double steerAngleRate_in);
        void set_state(State& stateIn);
        void set_parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in);

        void setReferenceControl(list<pair<Point2D, double>> referenceControl_in);
        /****************************************************************/
        /*** System methods *********************************************/
        /****************************************************************/
        void step(double Ts, double steerAngleCommand, double accelerationCommand);


};

