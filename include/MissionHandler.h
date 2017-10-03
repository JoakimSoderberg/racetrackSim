#pragma once
#include <list>
#include <vector>
#include "OptimalTrajectory.h"
#include "ObstacleMap.h"
//#include "ModelParameters.h"
#define M_PI 3.14159265358979323846

class MissionHandler
{
public:

    double Ts;


    MissionHandler(std::string opt_traj_ref_file);
    ~MissionHandler();

	//void set_optimal_trajectory(OptimalTrajectory* OptIn){OptTrajectory=OptIn;}

	std::vector<State> get_optimal_trajectory(){return OptTrajectory->get_optimal_trajectory_state_vector();} 

    list<WayPoint> getMissionPlan(int missionID)
    {
        return referenceWaypoints[missionID-1];
    }

	 list<WayPoint> getNextMissionPlan(int missionID)
    {
		return nextReferenceWaypoints[missionID-1];
    }

	void set_obastacle_map(ObstacleMap *ObstIn){obstacleMap=ObstIn;}

	//void refresh_way_point(State state);

	void set_mission_plan(bool safe_mode, State egoState, RectObstacle rectObstacle);
	//bool is_in_colution(State state, Parameters parameters);

private:
	int way_point_id;
    int numberOfMissions;
	OptimalTrajectory *OptTrajectory;
	ObstacleMap *obstacleMap;
    vector<list<WayPoint>> referenceWaypoints;
	vector<list<WayPoint>> nextReferenceWaypoints;
};
