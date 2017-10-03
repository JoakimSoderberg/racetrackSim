/****************************************************************/
/*** Simulation methods *****************************************/
/****************************************************************/
void	Model::step(double Ts, double steerAngleCommand, double accelerationCommand)
{
	/****************************************************************/
	/****** Propagate new states  ***********************************/
	/****************************************************************/

	// NOTE:
	// Assuming state has double beta2, beta3, heading, x, y;
	// Assuming parameters has double M1, L1, L2, L3;


	//Preclaculate some trigonometry for optimization.
	double tan_alpha = tan(steerAngleCommand);

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

	state.x += Ts * state.v * cos_beta3 * cos_beta2 * off_hitch_dynamics * cos_theta3;

	state.y += Ts * state.v * cos_beta3 * cos_beta2 * off_hitch_dynamics * sin_theta3;

	state.heading += Ts * state.v * sin_beta3 * cos_beta2 / L3 * off_hitch_dynamics;

	state.beta3 += Ts * state.v * cos_beta2 * (1 / L2 * (tan_beta2 - M1 / L1 * tan_alpha) - sin_beta3 / L3 * off_hitch_dynamics);

	state.beta2 += Ts * state.v * (tan_alpha / L1 - sin_beta2 / L2 + M1 / (L1 * L2) * cos_beta2 * tan_alpha);

	//double Gslip = 1 / (1 + state.v / parameters.vCH);

	/****************************************************************/
	/****** Angle propagation with restrictions  ********************/
	/****************************************************************/

	double steerAngleRate = 1 / parameters.Td * (steerAngleCommand - state.steerAngle);

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

	state.v = state.v + Ts*state.a;

	state.a = state.a + Ts / parameters.Ta * (accelerationCommand - state.a);

}