#pragma once
#include "globaldefines.h"

class Parameters
{
    public:
        double L;				// Wheel base distance
        double vCH;				// Slip speed
        double Td;				// Delay in steering command
        double Ta;				// Delay in acceleration command
        double maxSteerAngle;
        double maxSteerAngleRate;
        double maxAccelearion;
        double maxDeceleration;

        double width;
        double length;

        Parameters();
        Parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in);
};
