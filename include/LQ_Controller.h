#pragma once

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <windows.h>
#include <conio.h>
#include <vector>

#define _WIN64
#ifdef _WIN64
#define _USE_MATH_DEFINES
#endif
#include <math.h>

using namespace std;

//Struct used to keep structure of parameters used by the LQ_controller class.
struct RegulatorData {
	float Kc_beta2, Kc_beta3, alpha, beta2, beta3;
};



class LQ_Controller {

public:
	/* Constructor: 
	Input_file should be a txt-file with 5 columns corresponding to beta2 beta3 kc2 kc3 and alpha. 
	The number of rows don't matter but the resolution in steer_angle should be 0.01 rad.
	The first and the last element of alpha should be -max_steer_angle_in and max_steer_angle_in. */
	LQ_Controller();
	LQ_Controller(double max_steer_angle_in, string input_file);

	/* Function that given a desired steer angle(radians) and states(radians) of the vehicle 
	will return a stabilizing feedback control law. alpha = -Lx + u_d (deg) and alpha is 
	saturated to be within the boundaries -max_steer_angle < alpha < max_steer_angle_in */
	double Calculate_control(double desired_steer_angle, double beta2, double beta3);

private:
	/* Vector used to store controller data for the Linear quadratic controller */
	vector<RegulatorData> regulator_data_struckt_list;

	/* Maximal steering angle in degrees */
	double max_steer_angle;


	/* Variable used to find most suitable equilibrium point given (Used in Calculate_control) */
	double steer_angle_resulution;
};