#pragma once

#include <iostream>
#include <list>
#include <random>

#include "Geometry.h"
#include "PlannerTypes.h"

using namespace std;

class SamplingStrategies
{
  /****************************************************************/
  /*** Setup of random number generator ***************************/
  /****************************************************************/
  default_random_engine generator;
  normal_distribution<double> stdGaussianDist;
  uniform_real_distribution<double> stdUniformDist;

 public:
  /****************************************************************/
  /*** Constructor destructor *************************************/
  /****************************************************************/
  SamplingStrategies ();   
  ~SamplingStrategies ();

  /****************************************************************/
  /*** Sampling mehods  *******************************************/
  /****************************************************************/
  Point2D drawArcSample(double x, double y, double r0, double theta0, 
			double rStd, double thetaStd);

  Point2D drawUniformOnRectangle(double x, double y, double width, 
				 double length);

  Point2D drawGaussian(double x, double y, 
		       double stdx, double stdy);

  Point2D drawGoalSampleWithHeading(double x, double y,
						  double heading);
  
  void Safety_mode_sampling(pair<Sample, Sample> &randomSample, 
			     pair<Sample, Sample> &goalSample, State &egoState, WayPoint &goalPoint, WayPoint &nextGoalPoint);
 

  void Racing_mode_sampling(pair<Sample, Sample> &randomSample, 
				pair<Sample, Sample> &goalSample, State &egoState, WayPoint &goalPoint, WayPoint &nextGoalPoint, bool goalReached);

};
