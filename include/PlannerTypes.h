#pragma once

#include <set>
#include <list>
#include <limits>

#include "Geometry.h"

using namespace std;

typedef enum{feasable, nonFeasable, partlyFeasable 
}pathFeasability;

typedef enum{positionConstrained, headingConstrained
}WayPointType;

typedef enum{forwardDirection, reverseDirection, stopDrive
}Direction;

typedef enum{Safety_Behaviour, Racing_Behaviour
}Behaviour;

class Sample
{
	public:
		Point2D samplePoint;
		Direction direction; 
		double velocity;
		bool active;

		Sample()
		{
			samplePoint = Point2D();
			direction	= Direction::forwardDirection;
			active = false;
		}
		Sample(Point2D samplePoint_in, Direction direction_in)
		{
			samplePoint = samplePoint_in;
			direction	= direction_in;
			active = true;
		}
		Sample(Point2D samplePoint_in, Direction direction_in, double v_in)
		{
			samplePoint = samplePoint_in;
			direction	= direction_in;
			velocity	= v_in;
			active = true;
		}
};

class State
{
	public:
		double x ,y;			// Position of model
		double beta2, beta3;
		double v;				// Velocity of model
		double a;				// Acceleration of model
		double heading;			// Heading of model
		double steerAngle;		// Heading of model
		double steerAngleRate;  // Heading of model
		

	State();
	State(const State& stateIn);
	State(double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in);
	void operator=(const State& other);
};

class ReferenceTrajectoryState
{
public:
	double x;
	double y;
	double theta;

	ReferenceTrajectoryState();
	ReferenceTrajectoryState(double x_in, double y_in, double theta_in);
};

class Trajectory
{
	public:
        list<State> stateList;
		double cost;

		Trajectory()
		{
            //stateList = new list<State>;
			cost = 0;
		}

		Trajectory(Trajectory &trajIn)
		{
			stateList = trajIn.stateList;
		}


		~Trajectory()
		{
//			 if (stateList)
//			 {
//				 stateList->clear();
//				 delete stateList;
//			 }
			 
		}

        State& getEndState() {return stateList.back();}
		double evaluateCost(){return cost;}
};

class WayPoint
{
	public:
		int ID;
		Point2D wp;
		double heading;
		double exitDistance;
		double velocity;
		WayPointType type;
		Behaviour entryBehaviour;
		Behaviour exitBehaviour;
		double width;
		double length;

		WayPoint()
		{
			ID = 1;
			wp.x = 0;
			wp.y = 0;
			velocity = 0;
			entryBehaviour = Behaviour::Safety_Behaviour;
			exitBehaviour = Behaviour::Safety_Behaviour;
		}
		WayPoint(double x_in, double y_in, double v_in,double heading_in, int ID_in, WayPointType type_in)
		{
			type = type_in;
			wp.x = x_in;
			wp.y = y_in;
			ID = ID_in;
			velocity = v_in;
			heading=heading_in;
			entryBehaviour = Behaviour::Safety_Behaviour;
			exitBehaviour = Behaviour::Safety_Behaviour;
		}
		WayPoint(double x_in, double y_in, double v_in, int ID_in, Behaviour entryBehaviour_in, Behaviour exitBehaviour_in, double exitDistance_in)
		{
			type = positionConstrained;
			wp.x = x_in;
			wp.y = y_in;
			ID = ID_in;
			velocity = v_in;
			exitDistance = exitDistance_in;
			entryBehaviour = entryBehaviour_in;
			exitBehaviour = exitBehaviour_in;
		}

	
		WayPoint(double x_in, double y_in, double v_in, double heading_in, int ID_in, Behaviour entryBehaviour_in, Behaviour exitBehaviour_in, double exitDistance_in)
		{
			type = positionConstrained;
			wp.x = x_in;
			wp.y = y_in;
			velocity = v_in;
			heading = heading_in;
			exitDistance = exitDistance_in;
			ID = ID_in;
			entryBehaviour = entryBehaviour_in;
			exitBehaviour = exitBehaviour_in;
		}
}; 


class Node {
            
		Point2D stateKey;
		double vRef;

        Node *parent;
        State state;
        set<Node*> children;

		Direction direction;
		bool safe;
		bool markedToKeep;
        double costFromParent;
        double costFromRoot;

		double lowerBoundCost;
		double upperBoundCost;

		bool root;
        Trajectory trajFromParent;

    public:

		//debug
		bool isSampleNode; 
        
        Node();      
 
        Node(const Node &vertexIn);

		~Node();
       
        //&State& getState() {return *state;}
              
        State& getState() {return state;}
		double getLowerBoundCost(){return lowerBoundCost;};
        Node& getParent() {return *parent;}
		void setParent(Node& nodeIn){parent = &nodeIn;}
		void setState(State& stateIn)
		{
            //if(state)
                //delete state;

            state = stateIn;//new State(stateIn);
		}

		void setTrajectory(Trajectory& trajectoryIn)
		{
//            if(trajFromParent)
//                delete trajFromParent;

            trajFromParent = trajectoryIn;
		}

		void addChild(Node* nodeIn){children.insert(nodeIn);}

        Direction   getDirection(){return direction;};
        Point2D     getStateKey(){return stateKey;}
		double		getVelocityRef(){return vRef;};
        void        setStateKey(Point2D p){stateKey = p;};

        Trajectory& getTrajectory(){return trajFromParent;}

		bool isRoot() {return root;}
		bool isSafe() {return safe;};
		void setSafe(){safe = true;};
		void setUnSafe(){safe = false;};
		void setDirection(Direction dir_in){direction = dir_in;};

		bool node_has_no_childs();
        
        double getCost() {return costFromRoot;}
    
        friend class Planner;
    };
