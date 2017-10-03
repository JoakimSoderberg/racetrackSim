
#include "ModelTrailer.h"

using namespace std;

/****************************************************************/
/*** Model class: Implementation of the system model that will  */
/*** be controled                      **************************/
/****************************************************************/

/****************************************************************/
/*** Constructor Destructor  ************************************/
/****************************************************************/
ModelTrailer::ModelTrailer (int ID)
{   
	debugLogg = false;
    state = State();
    parameters = Parameters();
    controller = new PurePursuitController();
    modelID = ID;

    controller->setState(&state);
    controller->setParameters(&parameters);

	stabilizing = false;

}

ModelTrailer::ModelTrailer (int ID, double lookAheadDistance,bool stabilizing_in)
{   
	debugLogg = false;
    state = State();
    parameters = Parameters();
    controller = new PurePursuitController(lookAheadDistance);
    modelID = ID;

    controller->setState(&state);
    controller->setParameters(&parameters);

	stabilizing = stabilizing_in;

	if (stabilizing)
	{
		//LQ-controller 
		string LQ_input_file_name = "beta23kc32alpha.txt";
		double max_steer_angle_in_deg = parameters.maxSteerAngle;
		LQ_controller = LQ_Controller(max_steer_angle_in_deg, LQ_input_file_name);
	}
}

ModelTrailer::~ModelTrailer()
{}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void	ModelTrailer::set_state(double x_in, double y_in, double v_in, double a_in, double heading_in, double yawRate_in, double steerAngle_in, double steerAngleRate_in)
{
    state = State(x_in, y_in, v_in, a_in, heading_in, steerAngle_in, steerAngleRate_in);
}

void	ModelTrailer::set_state(State& stateIn)
{
    state = stateIn;
}

void	ModelTrailer::set_parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in)
{
    parameters = Parameters(L_in, vCH_in, Td_in, Ta_in, maxSteerAngle_in, maxSteerAngleRate_in, maxAccelearion_in, maxDeceleration_in);
}

/****************************************************************/
/*** Simulation methods *****************************************/
/****************************************************************/
void	ModelTrailer::step(double Ts, double steerAngleCommand, double accelerationCommand)
{
	double desired_steerAngleCommand = steerAngleCommand;

	if (state.vx < 0)
	{
		if (stabilizing)
		{
			desired_steerAngleCommand = LQ_controller.Calculate_control(desired_steerAngleCommand, state.beta2, state.beta3);
		}
	}
	

    /****************************************************************/
    /****** Propagate new states  ***********************************/
    /****************************************************************/

    // NOTE:
    // Assuming state has double beta2, beta3, heading, x, y;
    // Assuming parameters has double M1, L1, L2, L3;


    //Preclaculate some trigonometry for optimization.
    double tan_alpha = tan(state.steerAngle);

    double tan_beta2 = tan(state.beta2);
    double cos_beta2 = cos(state.beta2);
    double sin_beta2 = sin(state.beta2);

    double tan_beta3 = tan(state.beta3);
    double cos_beta3 = cos(state.beta3);
    double sin_beta3 = sin(state.beta3);

    double cos_theta3 = cos(state.heading);
    double sin_theta3 = sin(state.heading);

    //Just for simplicity
    double M1 = parameters.M1;
    double L1 = parameters.L1;
    double L2 = parameters.L2;
    double L3 = parameters.L3;

    double off_hitch_dynamics = 1 + M1 / L1 * tan_beta2 * tan_alpha;



    //Euler forward approximation of derivetives.
    //x_dot = f(x,u) => x_t+1 = x_t + f(x_t,u_t)*Ts

    state.x += Ts * state.vx * cos_beta3 * cos_beta2 * off_hitch_dynamics * cos_theta3;

    state.y += Ts * state.vx * cos_beta3 * cos_beta2 * off_hitch_dynamics * sin_theta3;

    state.heading += Ts * state.vx * sin_beta3 * cos_beta2 / L3 * off_hitch_dynamics;

    state.beta3 += Ts * state.vx * cos_beta2 * (1 / L2 * (tan_beta2 - M1 / L1 * tan_alpha) - sin_beta3 / L3 * off_hitch_dynamics);

    state.beta2 += Ts * state.vx * (tan_alpha / L1 - sin_beta2 / L2 + M1 / (L1 * L2) * cos_beta2 * tan_alpha);


    /****************************************************************/
    /****** Angle propagation with restrictions  ********************/
    /****************************************************************/
	//SOMETHING FUZZY
	double steerAngleRate = 1 / parameters.Td * (desired_steerAngleCommand - state.steerAngle);

    if (abs(steerAngleRate) > parameters.maxSteerAngleRate)
    {
        if (steerAngleRate < 0)
            steerAngleRate = -parameters.maxSteerAngleRate;
        else
            steerAngleRate = parameters.maxSteerAngleRate;
    }

    state.steerAngleRate = steerAngleRate;

    state.steerAngle = state.steerAngle + Ts * steerAngleRate;

    if (abs(state.steerAngle) > parameters.maxSteerAngle)
    {

        if (state.steerAngle < 0)
            state.steerAngle = -parameters.maxSteerAngle;
        else
            state.steerAngle = parameters.maxSteerAngle;
    }


    /****************************************************************/
    /****************************************************************/
    /****************************************************************/

    state.vx = state.vx + Ts*state.a;

    state.a = state.a + Ts / parameters.Ta * (accelerationCommand - state.a);

    if (abs(state.a) > parameters.maxAccelearion)
    {
        if (state.a < 0)
            state.a = -parameters.maxAccelearion;
        else
            state.a =  parameters.maxAccelearion;
    }

}



