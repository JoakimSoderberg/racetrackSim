#pragma once

#include "Model.h"

/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/
class Model2axis : public Model
{
    private:

    public:

        /****************************************************************/
        /*** Constructor Destructor  ************************************/
        /****************************************************************/
        Model2axis (int ID);
        Model2axis (int ID, double lookAheadDistance);
        ~Model2axis();

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

