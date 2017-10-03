#pragma once

#include "Model.h"
#include "LQ_Controller.h"

/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/
class ModelTrailer : public Model
{
    private:
		LQ_Controller LQ_controller;
		bool stabilizing;

    public:

        /****************************************************************/
        /*** Constructor Destructor  ************************************/
        /****************************************************************/
        ModelTrailer (int ID);
        ModelTrailer (int ID, double lookAheadDistance, bool stabilizing_in);
        ~ModelTrailer();

        /****************************************************************/
        /*** Set methods ************************************************/
        /****************************************************************/
        void set_state(double x_in, double y_in, double v_in, double a_in, double heading_in, double yawRate_in, double steerAngle_in, double steerAngleRate_in);
        void set_state(State& stateIn);
        void set_parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in);

        /****************************************************************/
        /*** System methods *********************************************/
        /****************************************************************/
        void step(double Ts, double steerAngleCommand, double accelerationCommand);


};

