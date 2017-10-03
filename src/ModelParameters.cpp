#include "ModelParameters.h"

Parameters::Parameters()
{
    //Standard parameters from MITs Darpa car
	L					= (f_c + c_b) * 100;	// Wheel base distance
    vCH					= 20;		// Slip speed
    Td					= 0.02;		// Delay in steering command
    Ta					= 0.3;		// Delay in acceleration command
    maxSteerAngle		= 0.5435;	//u_max
	maxSteerAngleRate   = 1.63294;	//u_dot_max
    maxAccelearion		= 1.8;		//a_max
    maxDeceleration		= -6.0;		//-a_max

    width	= 2*w*100;
    length	= 100*(f_c+c_b);
}

Parameters::Parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in)
{
    //Standard parameters from MITs Darpa car
    L					= L_in;	// Wheel base distance
    vCH					= vCH_in;		// Slip speed
    Td					= Td_in;		// Delay in steering command
    Ta					= Ta_in;		// Delay in acceleration command
    maxSteerAngle		= maxSteerAngle_in;
	maxSteerAngleRate   = maxSteerAngleRate_in;
    maxAccelearion		= maxAccelearion_in;
    maxDeceleration		= maxDeceleration_in;
}
