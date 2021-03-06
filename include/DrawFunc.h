#ifndef __DRAW_FUNC_INCLUDED__
#define __DRAW_FUNC_INCLUDED__

#ifdef __GNUG__

#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#else 

#include <freeglut.h>
#include <gl.h>
#include <glu.h>

#endif

#include <iostream>
#include <vector>
#include <math.h>

#include "RRTPlanner.h"
#include "Model.h"
//#include "msg/mock_lcm/grid_map.hpp"

using namespace std;


void drawCircle(double radius, double x, double y, double z, int step, GLfloat color[]);
void drawSolidCircle(double radius, double x, double y, double z, int step, GLfloat color[]);
void drawTrianlge(double heading, double x, double y, double base, GLfloat color[]);
void drawLine(double x, double y, double z, double xe, double ye, double ze, GLfloat color[]);
void drawWPTrajectory(list<WayPoint> referenceTrajectory, double scale, double drawOffset_x, double drawOffset_y, GLfloat color[], GLfloat color2[]);
void drawTreeForward(list<Node*> *nodeList,  double SCENE_SCALE, double drawOffset_x, double drawOffset_y, bool DRAW_CONTROL, bool DRAW_TRAJ);
void drawTreeReverse(list<Node*> *nodeList,  double SCENE_SCALE, double drawOffset_x, double drawOffset_y,bool DRAW_CONTROL, bool DRAW_TRAJ);
void drawSafeNodes(list<Node*> *nodeList, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
void drawLowCostPath(list<Node*> *nodeList, Node& lowCostNode, Point2D commitedControlPoint, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
void drawRoadCenter(vector<Point2D> &centerOfRoad, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
void putText(string ss, int x, int y, int width, int height);
void putText3D(string s, int x, int y, int z);
void drawRectObstacle(list<RectObstacle> *staticObstacles, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
//void drawCostMap(mock_lcm::grid_map  *lcmMap, State egoState, double SCENE_SCALE, double drawOffset_x, double drawOffset_y);
void drawOptimalTrajectorylist(std::vector<State> state_for_opt_traj_vector, double scale, double drawOffset_x, double drawOffset_y, GLfloat color[], GLfloat color2[]);
#endif
