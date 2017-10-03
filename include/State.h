#pragma once
#define _WIN64		//FOR OSKAR
//#define UBUNTU	//FOR NIKLAS
#ifdef _WIN64
	#define _CRT_SECURE_NO_DEPRECATE
	#define _CRT_SECURE_NO_WARNINGS
	#pragma warning (disable : 4996) //OL Added bored because of all VS warnings from double to GLfloat type converters.
#endif

class State
{
    public:
        double x ,y;			// Position of model
        double vx, vy;				// Velocity of model
        double a;				// Acceleration of model
        double heading;			// Heading of model
        double steerAngle;		// Heading of model
        double steerAngleRate;  // Heading of model
        double yawRate;
        double beta2;
        double beta3;



    State();
    State(const State& stateIn);
    State(double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in);
    State(double x_in, double y_in, double vx_in, double vy_in, double a_in, double heading_in, double yawRate_in, double steerAngle_in, double steerAngleRate_in);

    void operator=(const State& other);
};
