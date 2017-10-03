#include "Model4axis.h"

using namespace std;

/****************************************************************/
/*** Model4axis class: Wrapper for generated simulink         ***/
/***                   4axis model              *****************/
/****************************************************************/

/****************************************************************/
/*** Constructor Destructor  ************************************/
/****************************************************************/
Model4axis::Model4axis (int ID)
{   debugLogg = false;
    state = State();
    parameters = Parameters();
    controller = new PurePursuitController();
    modelID = ID;

    controller->setState(&state);
    controller->setParameters(&parameters);

}

Model4axis::Model4axis (int ID, double lookAheadDistance)
{
    debugLogg = false;
    state = State();
    parameters = Parameters();
    controller = new PurePursuitController(lookAheadDistance);
    modelID = ID;

    controller->setState(&state);
    controller->setParameters(&parameters);

}

Model4axis::~Model4axis()
{
}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void	Model4axis::set_state(double x_in, double y_in, double vx_in, double vy_in, double a_in, double heading_in, double yawRate_in, double steerAngle_in, double steerAngleRate_in)
{
    state = State(x_in, y_in, vx_in, vy_in, a_in, heading_in, yawRate_in, steerAngle_in, steerAngleRate_in);

    /****************************************************************/
    /*** Communicate to simulink code  ******************************/
    /****************************************************************/
     //[x x_dot y y_dot heading yawrate]
//    trin_feedback2_U.InitModelState[0] = state.x;
//    trin_feedback2_U.InitModelState[1] = state.y;
//    trin_feedback2_U.InitModelState[2] = state.heading;
//    trin_feedback2_U.InitModelState[3] = state.v;
//    trin_feedback2_U.InitModelState[4] = state.yawRate;

//     trin_feedback2_U.resetModel        = true;
//     //Do two steps so the init values propagates through the unit delay in the simulink model
//     trin_feedback2_step();
//     trin_feedback2_step();
//     trin_feedback2_U.resetModel = false;

}

void	Model4axis::set_state(State& stateIn)
{

    /****************************************************************/
    /*** Communicate to simulink code  ******************************/
    /****************************************************************/
     //[x x_dot y y_dot heading yawrate]
//     trin_feedback2_U.InitModelState[0] = stateIn.x;
//     trin_feedback2_U.InitModelState[1] = stateIn.y;
//     trin_feedback2_U.InitModelState[2] = stateIn.heading;
//     trin_feedback2_U.InitModelState[3] = stateIn.v;
//     trin_feedback2_U.InitModelState[4] = stateIn.yawRate;

//     trin_feedback2_U.resetModel        = true;
//     //Do two steps so the init values propagates through the unit delay in the simulink model
//     trin_feedback2_step();
//     trin_feedback2_step();
//     trin_feedback2_U.resetModel = false;

     state = stateIn;
}
/*
void	Model4axis::setReferenceControl(list<pair<Point2D, double>> referenceControl_in)
{

    FILE * pFile;

    pFile = fopen ("myfile.txt","w");


    tRTDB_WAYPOINT_STR wp;

    tRTDB_WAYPOINT_STR traj[100];
    int numWp = 0;
    int numNodes = referenceControl_in.size();

    for(int i = 0; i < numNodes; i++)
    {
        trin_feedback2_U.path_astr[i] = wp;
    }

    list<pair<Point2D, double>>::const_iterator iter;
    double x2       = 0;
    double y2       = 0;
    double distance = 0;

    for(iter=referenceControl_in.begin(); iter!=referenceControl_in.end(); iter++)
    {
        traj[numWp].ti_F64          = 1;
        traj[numWp].x_F64           = (*iter).first.x;
        traj[numWp].y_F64           = (*iter).first.y;
        traj[numWp].distance_F32    = distance;

        distance += sqrt((traj[numWp].x_F64-x2)*(traj[numWp].x_F64-x2) + (traj[numWp].y_F64-y2)*(traj[numWp].y_F64-y2));

        x2 = traj[numWp].x_F64;
        y2 = traj[numWp].y_F64;

        numWp++;
    }



    //calculate heading of the trajetory
    for(int i = 1; i < numNodes; i++)
    {
        traj[i-1].heading_F32 = atan2(traj[i].y_F64 - traj[i-1].y_F64, traj[i].x_F64 - traj[i-1].x_F64);

        traj[i-1].curvature_F32 = 2*((traj[i].x_F64-traj[i-1].x_F64)*(traj[i+1].y_F64-traj[i].y_F64)-(traj[i].y_F64-traj[i-1].y_F64)*(traj[i+1].x_F64-traj[i].x_F64)) /
                sqrt( ((traj[i].x_F64-traj[i-1].x_F64)*(traj[i].x_F64-traj[i-1].x_F64)+(traj[i].y_F64-traj[i-1].y_F64)*(traj[i].y_F64-traj[i-1].y_F64))*((traj[i+1].x_F64-traj[i].x_F64)*(traj[i+1].x_F64-traj[i].x_F64)+(traj[i+1].y_F64-traj[i].y_F64)*(traj[i+1].y_F64-traj[i].y_F64))*((traj[i-1].x_F64-traj[i+1].x_F64)*(traj[i-1].x_F64-traj[i+1].x_F64)+(traj[i-1].y_F64-traj[i+1].y_F64)*(traj[i-1].y_F64-traj[i+1].y_F64)) );
    }

    traj[numNodes-1].heading_F32    = traj[numNodes-2].heading_F32;
    traj[numNodes-1].curvature_F32  = traj[numNodes-2].curvature_F32;

    for(int i = 0; i < numNodes; i++)
    {
        trin_feedback2_U.path_astr[i] = traj[i];
        fprintf (pFile, "Wp#: %d\n Time: %f \n X: %f \n Y: %f \n Heading: %f \n Curvature: %f \n Distance: %f \n\n",i, traj[i].ti_F64, traj[i].x_F64, traj[i].y_F64, traj[i].heading_F32, traj[i].curvature_F32, traj[i].distance_F32 );
        //printf ("Time: %f \n X: %f \n Y: %f \n Heading: %f \n Curvature: %f \n Distance: %f \n\n", (double)traj[i].ti_F64, traj[i].x_F64, traj[i].y_F64, traj[i].heading_F32, traj[i].curvature_F32, traj[i].distance_F32 );

    }

    fclose (pFile);

}

void	Model4axis::set_parameters(double L_in, double vCH_in, double Td_in, double Ta_in, double maxSteerAngle_in, double maxSteerAngleRate_in, double maxAccelearion_in, double maxDeceleration_in)
{

}

/****************************************************************/
/*** Simulation methods *****************************************/
/****************************************************************/
/*
void	Model4axis::step(double Ts, double steerAngleCommand, double accelerationCommand)
{
//    trin_feedback2_U.referenceSpeed_F64 = 10;
//    trin_feedback2_step();

//    //[x x_dot y y_dot heading yawrate]
//    state.x         = trin_feedback2_Y.ztOut_aF64[0];
//    state.y         = trin_feedback2_Y.ztOut_aF64[2];
//    state.v         = sqrt(trin_feedback2_Y.ztOut_aF64[1]*trin_feedback2_Y.ztOut_aF64[1] + trin_feedback2_Y.ztOut_aF64[3]*trin_feedback2_Y.ztOut_aF64[3]);
//    state.heading   = trin_feedback2_Y.ztOut_aF64[4];
//    state.yawRate   = trin_feedback2_Y.ztOut_aF64[5];

    axis4Model_U.resetModel = 0.04;
    axis4Model_U.referenceSpeed_F64 = 10;
    axis4Model_U.steerAngle = steerAngleCommand;

    axis4Model_U.ztIn_aF64[0] = state.x;
    axis4Model_U.ztIn_aF64[2] = state.y ;
    axis4Model_U.ztIn_aF64[1] = state.vx ;
    axis4Model_U.ztIn_aF64[3] = state.vy;
    axis4Model_U.ztIn_aF64[4] = state.heading;
    axis4Model_U.ztIn_aF64[5] = state.yawRate;

    axis4Model_step();

    //[x x_dot y y_dot heading yawrate]
    state.x         = axis4Model_Y.ztOut_aF64[0];
    state.y         = axis4Model_Y.ztOut_aF64[2];
    state.vx        = axis4Model_Y.ztOut_aF64[1];
    state.vy        = axis4Model_Y.ztOut_aF64[3];
    state.heading   = axis4Model_Y.ztOut_aF64[4];
    state.yawRate   = axis4Model_Y.ztOut_aF64[5];
}

*/