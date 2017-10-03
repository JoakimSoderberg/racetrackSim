#include "MapHandler.h"

MapHandler::MapHandler(ObstacleMap *oMapIn)
{
	mapType = MapType::LOCAL_OMAP;
	oMap = oMapIn;
}

/*MapHandler::MapHandler(mock_lcm::grid_map  *lcmMapIn)
{
mapType = MapType::LCM_COSTMAP;
lcmMap = lcmMapIn;

Eigen::Matrix<double,Eigen::Dynamic,2> circleCenters_(1,2);
circleCenters_(0,0) = 0.0;
circleCenters_(0,1) = 0.0;
modelTemplate       = CollisionCheckTemplate(circleCenters_);
costChecker         = CostChecker(modelTemplate);
costChecker.setCostMap(lcmMap);

}*/

double MapHandler::getCost(State &state, Parameters parameters,double time)
{
	if (mapType == MapType::LOCAL_OMAP)
		return getCostOmap(state, parameters, time);
	else
		//Not possible to be here.
		return DBL_MAX;
}

double MapHandler::getCostDynamic(State &state, Parameters parameters,double time)
{

	if (mapType == MapType::LOCAL_OMAP)
		return getCostOmapDynamic(state, parameters, time);
	else
		//Not possible to be here.
		return DBL_MAX;
}

/*double MapHandler::getCostLCMCost(State &state)
{

//Convert to local map coordinates
Point2D mapStateP;
Point2D checkStateP(state.x, state.y);
Point2D egoStateP(egoState->x, egoState->y);

mapStateP = checkStateP - egoStateP;
//mapStateP = mapStateP.rotate(-egoState->heading);

if(costChecker.checkCost(mapStateP.x, mapStateP.y, 0) != 0)
std::cout << costChecker.checkCost(mapStateP.x, mapStateP.y, 0) << " x: " << mapStateP.x << " y: " << mapStateP.y << std::endl;

return costChecker.checkCost(mapStateP.y, mapStateP.x, state.heading);
}*/


int MapHandler::getCostOmap(State &state, Parameters parameters, double time)
{
	/****************************************************************/
	/****************************************************************/
	/*** Do collision check in the obstacle map *********************/
	/****************************************************************/
	/****************************************************************/

	list<RectObstacle>::iterator obstacle;

	/****************************************************************/
	/*** Calculate current model perimiter **************************/
	/****************************************************************/
	RectObstacle modelPerimiter = RectObstacle(Point2D(state.x, state.y), parameters.width, parameters.length, state.heading,0,0);


	/****************************************************************/
	/*** Loop through all obstacles in oMap and check for        ****/
	/*** intersection                                            ****/
	/****************************************************************/

	//Static obstacles
	for(obstacle = oMap->staticObstacles.begin(); obstacle != oMap->staticObstacles.end(); obstacle++)
	{
		for(int i = 0; i < modelPerimiter.perimiter.size(); i++)
		{
			for(int j = 0; j < obstacle->perimiter.size(); j++)
			{
				Point2D point;
				// if(modelPerimiter.perimiter[i].getIntersection(obstacle->perimiter[j], Point2D()))
				if(modelPerimiter.perimiter[i].getIntersection(obstacle->perimiter[j], point))
				{
					return 255;
				}
			}
		}
	}
	return getCostOmapDynamic(state,parameters,time);

}


int MapHandler::getCostOmapDynamic(State &state, Parameters parameters,double time)
{

	/****************************************************************/
	/*** Do collision check in the obstacle map *********************/
	/****************************************************************/

	list<RectObstacle>::iterator obstacle;

	/****************************************************************/
	/*** Calculate current model perimiter **************************/
	/****************************************************************/
	RectObstacle modelPerimiter = RectObstacle(Point2D(state.x, state.y), parameters.width, parameters.length, state.heading,0,0);


	list<RectObstacle> dynamic_obstacle_map=oMap->predic_position_of_dynamic_obstacle(time);

	for(obstacle = dynamic_obstacle_map.begin(); obstacle != dynamic_obstacle_map.end(); obstacle++)
	{
		for(int i = 0; i < modelPerimiter.perimiter.size(); i++)
		{
			for(int j = 0; j < obstacle->perimiter.size(); j++)
			{
				Point2D point;
				if(modelPerimiter.perimiter[i].getIntersection(obstacle->perimiter[j], point))
				{
					return 255;
				}
			}
		}
	}

	return 0;
}
