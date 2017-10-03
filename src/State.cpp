#include "State.h"

State::State()
{
  x                 = 0;
  y                 = 0;
  vx                = 0;
  vy                = 0;
  a                 = 0;
  heading			= 0;
  yawRate           = 0;
  steerAngle		= 0;
  steerAngleRate	= 0;
}

State::State(double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in)
{
  x                 = x_in;
  y                 = y_in;
  vx                = v_in;
  vy                = 0;
  a                 = a_in;
  heading			= heading_in;
  yawRate           = 0;
  steerAngle		= steerAngle_in;
  steerAngleRate	= steerAngleRate_in;
}

State::State(double x_in, double y_in, double vx_in, double vy_in, double a_in, double heading_in, double yawRate_in, double steerAngle_in, double steerAngleRate_in)
{
  x                 = x_in;
  y                 = y_in;
  vx                = vx_in;
  vy                = vy_in;
  a                 = a_in;
  heading			= heading_in;
  yawRate           = yawRate_in;
  steerAngle		= steerAngle_in;
  steerAngleRate	= steerAngleRate_in;
}

State::State(const State& stateIn)
{
  x                 = stateIn.x;
  y                 = stateIn.y;
  vx                = stateIn.vx;
  vy                = stateIn.vy;
  a                 = stateIn.a;
  heading			= stateIn.heading;
  yawRate           = stateIn.yawRate;
  steerAngle		= stateIn.steerAngle;
  steerAngleRate	= stateIn.steerAngleRate;

}

void State::operator=(const State&  other)
{
  x                 = other.x;
  y                 = other.y;
  vx                = other.vx;
  vy                = other.vy;
  a                 = other.a;
  heading			= other.heading;
  yawRate           = other.yawRate;
  steerAngle		= other.steerAngle;
  steerAngleRate	= other.steerAngleRate;
  beta2             = other.beta2;
  beta3             = other.beta3;
}
