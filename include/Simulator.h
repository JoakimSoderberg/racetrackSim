#include "Model.h"
#include "ObstacleMap.h"

class Simulator
{
public:
  
    double Ts;
    
    
    Simulator(double Ts_in);
    ~Simulator();
    
    void step();

    Model* getSimulationModel()
    {
        return simulationModel;
    }
    ObstacleMap* getObstacleMap()
    {
        return obstacleMap;
    }

	void set_obstacle_map(ObstacleMap *obstacleMapIn){obstacleMap=obstacleMapIn;}

    void setControlTrajectory(list<pair<Point2D, double>> controlReference)
    {
        simulationModel->controller->setReferenceControl(controlReference);
    }
    
private:
    Model       *simulationModel;
    ObstacleMap *obstacleMap;
    
};
