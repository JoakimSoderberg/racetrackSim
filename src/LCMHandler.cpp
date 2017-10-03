#include "LCMHandler.h"

#ifdef UBUNTU

#include "util/Geography.hpp"


LCMHandler::LCMHandler() :
    lcmManager("udpm://239.255.76.67:7667?ttl=1"),
    egoInStream(lcmManager.registerInputDataStream<scania_lcm::tVEGO_CTRV_STRS>("vego_global")),
    mapInStream(lcmManager.registerInputDataStream<scania_lcm::tSITAW_DRIVE_MAP_STR >("cost_map")),
    btInStream(lcmManager.registerInputDataStream<scania_lcm::tSITAW_STATUS_STR>("bt_state")),
    controlOutStream(lcmManager.registerOutputDataStream<scania_lcm::tWAYPOINT_LIST_STR>("horizon")),
    SAABOutStream(lcmManager.registerOutputDataStream<iQMatic_commands::drive_request_t>("drive_request")),
    egoOutStream(lcmManager.registerOutputDataStream<scania_lcm::tVEGO_CTRV_STRS>("vego_global"))

{

//    // oMapInStream(lcmManager.registerInputDataStream<scania_lcm::tOMAP_GRID_STR>("omap_grid")),
//    lcmMap.gridMap_str.cost_aaS08.resize(500 , vector<int8_t>( 500 , 0 ));

//    for(int i = 0; i < 500; i++)
//        lcmMap.gridMap_str.cost_aaS08[300][i] = 127;


//    lcmMap.gridMap_str.cols_S32 = 500;
//    lcmMap.gridMap_str.rows_S32 = 500;
//    lcmMap.gridMap_str.cellSize_F32 = 0.4;

    laneCenter  = vector<scania_lcm::tSITAW_WAYPOINT_STR>();
    lcmMap      = scania_lcm::tSITAW_DRIVE_MAP_STR();

}

/****************************************************************/
/*** Initzialisation of communication   *************************/
/****************************************************************/

bool LCMHandler::waitForValidMessage(State &readState)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    while(!readEgoState(readState))
    {
        cout << "Waiting for valid LCM position..." << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return true;
}


/****************************************************************/
/*** Read functions      ****************************************/
/****************************************************************/
bool LCMHandler::readMapState()
{

    //read LCM objects to object vector
    //lcmManager.manage();

    int64_t t;
    if(mapInStream->numMsg() > 0)
    {
        lcmMap = mapInStream->getLatest(t);
        laneCenter = lcmMap.list_astr;
        roadReferenceID = lcmMap.referencePathId_S32;

        return true;
    }

    return false;

}

bool LCMHandler::readEgoState(State &readState)
{

    //read LCM objects to object vector
    lcmManager.manage();

    int64_t t;
    scania_lcm::tVEGO_CTRV_STRS lcmState = egoInStream->getLatest(t);

    if(lcmState.ss_U08 >= 250)
    {
        /* double x_in, double y_in, double v_in, double a_in, double heading_in, double steerAngle_in, double steerAngleRate_in
        /* EGO position CTRV [x y th v dth]        */
        readState.x         = lcmState.M_ctrv_aF64[0];//-50 + 0*6.499206249504017e+05;//lcmState.M_ctrv_aF64[0];
        readState.y         = lcmState.M_ctrv_aF64[1];//-10 + 0*6.561932001255457e+06;//lcmState.M_ctrv_aF64[1];
        readState.heading   = lcmState.M_ctrv_aF64[2];//0*M_PI/2; //*-0.892087741408216;//lcmState.M_ctrv_aF64[2];
        readState.vx        = lcmState.M_ctrv_aF64[3];//0;//lcmState.M_ctrv_aF64[3];
        readState.yawRate   = lcmState.M_ctrv_aF64[4];

        return true;
    }

    return false;
}

bool LCMHandler::readOmapState()
{
    const int OMAP_GRID_SIZE = 200;
    int64_t t;
    scania_lcm::tOMAP_GRID_STR omap = oMapInStream->getLatest(t);

    int omap_rows = OMAP_GRID_SIZE;
    int omap_cols = OMAP_GRID_SIZE;

    cv::Mat omap_mat(omap_rows,omap_cols,CV_8UC1);


    //THIS IS THE CORRECT ONE!! YAY
    //try 0,0 -> bottom right
    for(int i=0; i<omap_rows; ++i)  {
        for(int j=0; j<omap_cols; ++j) {
            //column major and 0,0 is bottom left
            int omap_idx = omap_cols*j + i;
            uchar omap_val = omap.grid_aaU08[omap_idx]; //column major
            omap_mat.at<uchar>(omap_mat.rows-1-i,j) = omap_val;
        }
    }
    cv::applyColorMap(omap_mat,omap_vis,cv::COLORMAP_JET);
//    cv::imshow("omap_cm_00@bl",omap_vis);
//     cv::waitKey();

}

bool    LCMHandler::readBehaviourState(scania_lcm::tSITAW_STATUS_STR& btStatus)
{
    //read LCM objects to object vector
    //lcmManager.manage();

    if(mapInStream->numMsg() > 0)
    {
        btStatus =  lcmMap.status_str;
       // cout << "ID: " << btStatus.referencePathId_S32 << " " << btStatus.onRoadStatus_B << " " << btStatus.openAreaStatus_B << endl;
        return true;
    }
    else
        return false;

}

void    LCMHandler::readLaneCenter(vector<Point2D> &laneCenter_v)
{
        for(double i = 0; i < 300; i+= 3)
        {
           scania_lcm::tSITAW_WAYPOINT_STR p;

           p.pose_str.coord_aF64[0] = 650027 + i;
           p.pose_str.coord_aF64[1] = 6562113;

           laneCenter.push_back(p);
        }

//    double r = 50;
//    double xc = 10;
//    double yc = -50;

//    laneCenter_v.clear();

//    for(double i = 0; i<M_PI; i+=0.1)
//    {
//        Point2D p =  Point2D(xc + r * sin(i), yc + r * cos(i));
//        laneCenter_v.push_back(p);
//    }
//    xc = 12.0790331216645;
//    yc = -39.9567575136640 -60;

//    for(double i = 1; i < 43; i+=5)
//    {
//        Point2D p =  Point2D(xc - i,  yc);
//        laneCenter_v.push_back(p);
//    }

//    xc = -33.920966878335500;
//    yc = -90 -60;
//    for(double i = 2*M_PI; i > M_PI; i -= 0.1)
//    {
//        Point2D p =  Point2D(xc + r * sin(i), yc + r * cos(i));
//        laneCenter_v.push_back(p);
//    }

//    for(double i = 0; i < 300; i+= 3)
//    {
//        Point2D p;
////        if(i < 20)
////            p =  Point2D(i, 0);
////        else
//            p =  Point2D(i, 0);
//        laneCenter_v.push_back(p);
//    }

//    laneCenter_v.resize(laneCenter.length_S16);

//    for(int i = 0; i < laneCenter.length_S16; i++)
//    {

//        laneCenter_v[i].x = laneCenter.pose_str.coord_aF64[0];
//        laneCenter_v[i].y = laneCenter.list_astr[i].y_F64;
//    }
}


/****************************************************************/
/*** Send functions      ****************************************/
/****************************************************************/
void LCMHandler::sendControlTrajectory(list<State> controlTrajectoryOut, double vRef)
{
    /****************************************************************/
    /*** Send to LCM ************************************************/
    /****************************************************************/

    if(controlTrajectoryOut.size() == 0)
        return;

    scania_lcm::tWAYPOINT_LIST_STR toSend;
    toSend.ti_S64       = (int64_t)clock();;

    list<State>::const_iterator wpIter;
    list<State>::const_iterator last_wpIter;

    double distance = 0;
    double distanceSinceLastPoint = 0;
    bool firstPoint = true;

    scania_lcm::tWAYPOINT_STR wp;

    int i = 0;
    for(wpIter=controlTrajectoryOut.begin(); wpIter!=controlTrajectoryOut.end(); ++wpIter)
    {
        i++;
        if(i > 100)
            break;        
        
        double x1 = (*wpIter).x;
        double y1 = (*wpIter).y;
        double x2;
        double y2;

        if(!firstPoint)
        {
            if((*last_wpIter).x == (*wpIter).x)
                int dhe = 0;

            x2          = (*last_wpIter).x;
            y2          = (*last_wpIter).y;
            double diff = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

            if(diff != 0)
            {
                distanceSinceLastPoint += diff;
                distance += diff;

                wp.x_F64            =   (*wpIter).x;
                wp.y_F64            =   (*wpIter).y;
                wp.curvature_F32    =   (fmod((*wpIter).heading -  (*last_wpIter).heading + M_PI,2*M_PI) - M_PI) / diff;
                wp.heading_F32      =   (*wpIter).heading;
                wp.distance_F32     =   distance;
                wp.v_ref_F32        =   (*wpIter).vx;

                if(distanceSinceLastPoint > 1)
                {
                    if(wp.v_ref_F32 == 0)
                        int hthe = 0;
                    toSend.list_astr.push_back(wp);

                    distanceSinceLastPoint = 0;
                }
            }
        }

        firstPoint = false;
        last_wpIter = wpIter;
    }

    if(toSend.list_astr.size() == 0)
        return;

    if(controlTrajectoryOut.back().vx == 0)
        toSend.list_astr.back().v_ref_F32 = 0;

    //always add last point!
   // toSend.list_astr.push_back(wp);

    toSend.length_S16 = toSend.list_astr.size();

    if(controlTrajectoryOut.size() > 0)
        controlOutStream->put(&toSend);

}

/****************************************************************/
/*** DEBUG Stuff                                  ***************/
/****************************************************************/
void LCMHandler::sendSAABStuff(Point2D p)
{
    util::EarthCoordinates ec = util::getEarthCoordinates(p.x,p.y,33,true);


    iQMatic_commands::drive_request_t toSend;


    iQMatic_commands::checkpoint_t wp;

    wp.lat = ec.wgs84_lat;
    wp.lon = ec.wgs84_lon;

    wp.target_id = 1;
    wp.target_type = iQMatic_commands::checkpoint_type_enum_t::ZONE;

    toSend.checkpoints.push_back(wp);

    toSend.checkpoint_count = toSend.checkpoints.size();
    SAABOutStream->put(&toSend);
}

void LCMHandler::sendEgo(State egoState)
{
    scania_lcm::tVEGO_CTRV_STRS ego;
    static long int time = 0;
    time++;

    ego.ti_S64 = time;
    ego.ss_U08 = 255;
    ego.utmZone_aU08[1] = 33;
    ego.utmZone_aU08[2] = 18;
    ego.M_ctrv_aF64[0] = egoState.x;
    ego.M_ctrv_aF64[1] = egoState.y;
    ego.M_ctrv_aF64[2] = egoState.heading;
    ego.M_ctrv_aF64[3] = egoState.vx;
    ego.M_ctrv_aF64[4] = egoState.yawRate;

    egoOutStream->put(&ego);
}
#endif