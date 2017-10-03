#include "SamplingStrategies.h"

SamplingStrategies::SamplingStrategies(){
}
SamplingStrategies::~SamplingStrategies(){
}

Point2D SamplingStrategies::drawArcSample(double x, double y, double r0, double theta0, double rStd, double thetaStd)
{

	//draw random numbers
	double nr		= stdGaussianDist(generator);
	double ntheta	= stdGaussianDist(generator);
	
	double r		= rStd*abs(nr) + r0;
	double theta	= thetaStd*ntheta + theta0;

	double sx = x + r*cos(theta);
	double sy = y + r*sin(theta);

	return Point2D(sx, sy);
}

Point2D SamplingStrategies::drawUniformOnRectangle(double x, double y, double width, double length)
{

	double sx		= (x-length/2) + stdUniformDist(generator)*((x+length/2)-(x-length/2));
	double sy		= (y-width/2) + stdUniformDist(generator)*((y+width/2)-(y-width/2));
	return Point2D(sx, sy);
}

Point2D SamplingStrategies::drawGaussian(double x, double y, double stdx, double stdy)
{
	double sx		= stdx*stdGaussianDist(generator) + x;
	double sy		= stdy*stdGaussianDist(generator) + y;
	return Point2D(sx, sy);
}

Point2D SamplingStrategies::drawGoalSampleWithHeading(double x, double y, double heading)
{

	//draw first sample infront waypoint
	Point2D sample_point  = drawArcSample(x, y, 10, heading, 10, 0.3);

	return sample_point;
}

void SamplingStrategies::Safety_mode_sampling(pair<Sample, Sample> &randomSample, pair<Sample, Sample> &goalSample, State &egoState, WayPoint &goalPoint, WayPoint &nextGoalPoint)
{
	bool next_goal_p_equal_to_goal_p = false;

	if (goalPoint.wp.y - nextGoalPoint.wp.y == 0 && goalPoint.wp.x - nextGoalPoint.wp.x == 0)
		next_goal_p_equal_to_goal_p = true;

	double rand = stdUniformDist(generator);

	double refangle= atan2(goalPoint.wp.y-egoState.y,goalPoint.wp.x-egoState.x);

	double dist = sqrt((goalPoint.wp.y-egoState.y)*(goalPoint.wp.y-egoState.y)+(goalPoint.wp.x-egoState.x)*(goalPoint.wp.x-egoState.x));

	if(rand < 0.5)
	{
        randomSample.first.samplePoint  = drawArcSample(egoState.x, egoState.y, 10 + dist/4, refangle, dist/5, 0.1*3.1415);
        randomSample.first.direction    = Direction::forwardDirection;
	}
	else
	{
		randomSample.first.samplePoint = drawArcSample(egoState.x, egoState.y, 10 + dist / 2, refangle, dist / 3, 0.1*3.1415);
		randomSample.first.direction = Direction::forwardDirection;
	}

	goalSample.first.samplePoint = drawGoalSampleWithHeading(goalPoint.wp.x, goalPoint.wp.y, goalPoint.heading);
	goalSample.first.direction = Direction::forwardDirection;
}


void SamplingStrategies::Racing_mode_sampling(pair<Sample, Sample> &randomSample, pair<Sample, Sample> &goalSample, State &egoState, WayPoint &goalPoint, WayPoint &nextGoalPoint,bool goalReached)
{
	double rand = stdUniformDist(generator);

	double dist = sqrt((goalPoint.wp.y-egoState.y)*(goalPoint.wp.y-egoState.y)+(goalPoint.wp.x-egoState.x)*(goalPoint.wp.x-egoState.x));

	bool next_goal_p_equal_to_goal_p=false;

	if(goalPoint.wp.y-nextGoalPoint.wp.y == 0 && goalPoint.wp.x-nextGoalPoint.wp.x==0)
		next_goal_p_equal_to_goal_p=true;

	if(!goalReached || next_goal_p_equal_to_goal_p)
	{
		//Fesible trajectory NOT found, look for trajectory to waypoint.
		if(rand < 0.5)
		{
			randomSample.first.samplePoint = drawArcSample(egoState.x, egoState.y, 10 +2 * dist / 3, goalPoint.heading, dist / 4, 0.3*3.1415);
			randomSample.first.direction = Direction::forwardDirection;
		}
		else
		{
			randomSample.first.samplePoint = drawArcSample(egoState.x, egoState.y, 10 + 1 * dist / 3, goalPoint.heading, dist / 4, 0.3*3.1415);
			randomSample.first.direction = Direction::forwardDirection;
		}

		goalSample.first.samplePoint = drawGoalSampleWithHeading(goalPoint.wp.x, goalPoint.wp.y, goalPoint.heading);
		goalSample.first.direction = Direction::forwardDirection;
	
	}
	else
	{
		// Fesible trajectory IS found, start looking on trajectory to next waypoint.
		// Devide search between current ant next waypoint.
		if(rand < 0.5)
		{
			//Look for trajectory to current waypoint.
			if(rand < 0.25)
			{
				randomSample.first.samplePoint = drawArcSample(egoState.x, egoState.y, 2 * dist / 3, goalPoint.heading, dist / 5, 0.2*3.1415);
				randomSample.first.direction = Direction::forwardDirection;
			}
			else
			{
				randomSample.first.samplePoint = drawArcSample(egoState.x, egoState.y, dist / 3, goalPoint.heading, dist / 4, 0.2*3.1415);
				randomSample.first.direction = Direction::forwardDirection;
			}

			goalSample.first.samplePoint = drawGoalSampleWithHeading(goalPoint.wp.x, goalPoint.wp.y, goalPoint.heading+(nextGoalPoint.heading - goalPoint.heading));
			goalSample.first.direction = Direction::forwardDirection;
		}
		else
		{ 
			double dist2 = sqrt((nextGoalPoint.wp.y-goalPoint.wp.y)*(nextGoalPoint.wp.y-goalPoint.wp.y)+
								(nextGoalPoint.wp.x-goalPoint.wp.x)*(nextGoalPoint.wp.x-goalPoint.wp.x));

			randomSample.first.samplePoint = drawArcSample(goalPoint.wp.x, goalPoint.wp.y, dist2 / 3, goalPoint.heading + (nextGoalPoint.heading - goalPoint.heading), dist2 / 3, 0.2*3.1415);
			randomSample.first.direction = Direction::forwardDirection;

			goalSample.first.samplePoint = drawGoalSampleWithHeading(nextGoalPoint.wp.x, nextGoalPoint.wp.y, nextGoalPoint.heading);
			goalSample.first.direction = Direction::forwardDirection;
		}
	}
}

