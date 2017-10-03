#include <cstddef> //Defines NULL
#include "PlannerTypes.h"

using namespace std;

State::State()
{
  x				= 0;
  y				= 0;
  v				= 0;
  a				= 0;
  heading			= 0;
  steerAngle		= 0;
  steerAngleRate	= 0;
}

State::State(double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in)
{
  x				= x_in;
  y				= y_in;
  v				= v_in;
  a				= a_in;
  heading			= heading_in;
  steerAngle		= steerAngle_in;
  steerAngleRate	= steerAngleRate_in;
}

State::State(const State& stateIn)
{
  x				= stateIn.x;
  y				= stateIn.y;
  v				= stateIn.v;
  a				= stateIn.a;
  heading			= stateIn.heading;
  steerAngle		= stateIn.steerAngle;
  steerAngleRate	= stateIn.steerAngleRate;

}

void State::operator=(const State&  other)
{
  x				= other.x;
  y				= other.y;
  v				= other.v;
  a				= other.a;
  heading			= other.heading;
  steerAngle		= other.steerAngle;
  steerAngleRate	= other.steerAngleRate;
}

ReferenceTrajectoryState::ReferenceTrajectoryState(){

	x=0;
	y=0;
	theta=0;

}


ReferenceTrajectoryState::ReferenceTrajectoryState(double x_in, double y_in, double theta_in){

	x=x_in;
	y=y_in;
	theta=theta_in;

}

Node::Node () 
{
    
  parent			= NULL;

 
  costFromParent	= 0.0;
  costFromRoot      = 0.0;

  direction		= Direction::forwardDirection;
  safe			= false;

  lowerBoundCost    = numeric_limits<double>::max();
  upperBoundCost	= numeric_limits<double>::max();

  isSampleNode	= false;
  root			= false;
  markedToKeep = false;
}

Node::~Node () 
{
    

  parent = NULL;

  children.clear();
}

Node::Node(const Node& nodeIn) 
{
    
  state = nodeIn.state;
  markedToKeep=false;
	
  stateKey = Point2D(nodeIn.stateKey.x, nodeIn.stateKey.y);
    
  direction		= nodeIn.direction;
  safe			= nodeIn.safe;     

  parent = nodeIn.parent;

  vRef = nodeIn.vRef;

  for (set<Node*>::const_iterator iter = nodeIn.children.begin(); iter != nodeIn.children.end(); iter++)
    children.insert(*iter);

  costFromParent = nodeIn.costFromParent;
  costFromRoot = nodeIn.costFromRoot;

  lowerBoundCost = nodeIn.lowerBoundCost;
  upperBoundCost = nodeIn.upperBoundCost;

  root = nodeIn.root;

  trajFromParent = nodeIn.trajFromParent;
}


bool Node::node_has_no_childs(){

	if(children.size()>2)
		return false;
	else 
		return true;
}