
#include "Model2axis.h"

using namespace std;

/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/

/****************************************************************/
/*** Constructor Destructor  ************************************/
/****************************************************************/
Model2axis::Model2axis (int ID)
{   debugLogg = false;
    state = State();
    parameters = Parameters();
    controller = new PurePursuitController();
    modelID = ID;

    controller->setState(&state);
    controller->setParameters(&parameters);

}

Model2axis::Model2axis (int ID, double lookAheadDistance)
{   debugLogg = false;
    state = State();
    parameters = Parameters();
    controller = new PurePursuitController(lookAheadDistance);
    modelID = ID;

    controller->setState(&state);
    controller->setParameters(&parameters);

}

Model2axis::~Model2axis()
{}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void	Model2axis::set_state(double x_in, double y_in, double v_in, double a_in, double heading_in, double yawRate_in, double steerAngle_in, double steerAngleRate_in)
{
    state = State(x_in, y_in, v_in, a_in, heading_in, steerAngle_in, steerAngleRate_in);
}

void	Model2axis::set_state(State& stateIn)
{
    state = stateIn;
}

void	Model2axis::set_parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in)
{
    parameters = Parameters(L_in, vCH_in, Td_in, Ta_in, maxSteerAngle_in, maxSteerAngleRate_in, maxAccelearion_in, maxDeceleration_in);
}

/****************************************************************/
/*** Simulation methods *****************************************/
/****************************************************************/
void	Model2axis::step(double Ts, double steerAngleCommand, double accelerationCommand)
{


    /****************************************************************/
    /****** Propagate new states  ***********************************/
    /****************************************************************/
    state.x = state.x + Ts * state.vx * cos( state.heading );
    state.y = state.y + Ts * state.vx * sin( state.heading );

    double Gslip = 1 / (1 + state.vx / parameters.vCH);
    state.heading = state.heading + Ts * state.vx / parameters.L * tan( state.steerAngle ) * Gslip;

    /****************************************************************/
    /****** Angle propagation with restrictions  ********************/
    /****************************************************************/

    double steerAngleRate = 1 / parameters.Td * ( steerAngleCommand - state.steerAngle );

    if( abs(steerAngleRate) >  parameters.maxSteerAngleRate )
    {

        if(steerAngleRate < 0)
            steerAngleRate = -parameters.maxSteerAngleRate;
        else
            steerAngleRate =  parameters.maxSteerAngleRate;
    }

    state.steerAngleRate = steerAngleRate;

    state.steerAngle = state.steerAngle + Ts * steerAngleRate;

    if( abs(state.steerAngle) >  parameters.maxSteerAngle )
    {

        if(state.steerAngle  < 0)
            state.steerAngle = -parameters.maxSteerAngle;
        else
            state.steerAngle =  parameters.maxSteerAngle;
    }

    /****************************************************************/
    /****************************************************************/
    /****************************************************************/

    state.vx = state.vx + Ts*state.a;

    state.a = state.a + Ts / parameters.Ta * ( accelerationCommand - state.a );

}


