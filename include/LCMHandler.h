#pragma once

#ifdef UBUNTU
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <lcm/lcm-cpp.hpp>
#include <time.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include "data/DataHandler.hpp"
#include "data/Streams.hpp"
#include "PlannerTypes.h"
#include "Geometry.h"

/****************************************************************/
/*** Include Prescan LCM messages  *****************************/
/****************************************************************/
//#include "msg/iqmatic/sim/static_object_list.hpp"
//#include "msg/mock_lcm/ego_state.hpp"
//#include "msg/mock_lcm/bt_status.hpp"
//#include "msg/mock_lcm/bt_acknowledgment.hpp"

/****************************************************************/
/*** Include Scania LCM messages  ******************************/
/****************************************************************/
#include "msg/scania_lcm/tWAYPOINT_LIST_STR.hpp"
#include "msg/scania_lcm/tVEGO_CTRV_STRS.hpp"
#include "msg/scania_lcm/tOMAP_GRID_STR.hpp"
#include "msg/scania_lcm/tSITAW_DRIVE_MAP_STR.hpp"
#include "msg/iQMatic_commands/drive_request_t.hpp"
#include "msg/iQMatic_commands/checkpoint_type_enum_t.hpp"

#include "msg/scania_lcm/tSITAW_STATUS_STR.hpp"

class LCMHandler
{

    public:
        /****************************************************************/
        /*** Constructor/Destructor   ***********************************/
        /****************************************************************/
        LCMHandler();
        ~LCMHandler();

        /****************************************************************/
        /*** Initzialisation of communication   *************************/
        /****************************************************************/
        bool    waitForValidMessage(State &readState);

        /****************************************************************/
        /*** Read functions      ****************************************/
        /****************************************************************/
        bool    readMapState();
        bool    readEgoState(State &readState);
        bool    readOmapState();
        void    readLaneCenter(vector<Point2D> &laneCenter_v);
        bool    readBehaviourState(scania_lcm::tSITAW_STATUS_STR& btStatus);


        /****************************************************************/
        /*** Send functions      ****************************************/
        /****************************************************************/
        void sendControlTrajectory(list<State> controlTrajectoryOut, double vRef);

        void sendSAABStuff(Point2D p);
        void sendEgo(State egoState);

        /****************************************************************/
        /*** Misc     ***************************************************/
        /****************************************************************/
        //for debug plot!!!
       // scania_lcm::tWAYPOINT_LIST_STR toSend;
        cv::Mat omap_vis;
        scania_lcm::tSITAW_GRID_STR*                getLCMMap(){return &lcmMap.gridMap_str;}
        vector<scania_lcm::tSITAW_WAYPOINT_STR>*    getRoadReference(){return &laneCenter;}
        int*                                        getRoadReferenceID(){return &roadReferenceID;}


    private:
        scania_lcm::tSITAW_DRIVE_MAP_STR            lcmMap;
        vector<scania_lcm::tSITAW_WAYPOINT_STR>     laneCenter;
        int                                         roadReferenceID;

        LCMManager lcmManager;
        std::shared_ptr<InputStream<scania_lcm::tVEGO_CTRV_STRS>>           egoInStream;
        std::shared_ptr<InputStream<scania_lcm::tSITAW_DRIVE_MAP_STR >>     mapInStream;
        std::shared_ptr<InputStream<scania_lcm::tOMAP_GRID_STR>>            oMapInStream;
        std::shared_ptr<InputStream<scania_lcm::tSITAW_STATUS_STR>>                   btInStream;

        std::shared_ptr<OutputStream<scania_lcm::tVEGO_CTRV_STRS>>          egoOutStream;

        std::shared_ptr<OutputStream<scania_lcm::tWAYPOINT_LIST_STR>>       controlOutStream;
        std::shared_ptr<OutputStream<iQMatic_commands::drive_request_t>>    SAABOutStream;
};

#endif

