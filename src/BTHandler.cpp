#include "BTHandler.h"

#ifdef UBUNTU
void BTHandler::updateBehaviourTree()
{
    static int currentPathId = -99;

    scania_lcm::tSITAW_STATUS_STR btStatus;
    btStatus.onRoadStatus_B     = false;
    btStatus.openAreaStatus_B   = false;
    int referencePathId_S32     = -99;

   // cout << "Entering BT: " << endl;
    if(lcmHandler->readBehaviourState(btStatus) && btStatus.openAreaStatus_B)
    {
         //cout << " Read new list. Currnet ID: "<< currentPathId <<  endl;
        /****************************************************************/
        /*** Check if we received a new waypoint list    ****************/
        /****************************************************************/
        referencePathId_S32 = mapHandler->getReferencePathID();
        if(currentPathId != referencePathId_S32)
        {
            cout << "  New List: " << referencePathId_S32 << endl;
            currentPathId = referencePathId_S32;

            vector<scania_lcm::tSITAW_WAYPOINT_STR> recivedWayPointList;
            recivedWayPointList = mapHandler->getRoadCenterReference();

            list<WayPoint> wayPointList;

            WayPoint egoPoint = WayPoint(planner->egoState->x, planner->egoState->y, planner->egoState->vx, 0, Behaviour::normalParking, Behaviour::normalParking, 1);
            wayPointList.push_back(egoPoint);

            for(int i = 0; i < recivedWayPointList.size(); i++)
            {

                cout << "   in loop: " <<  i << endl;
                scania_lcm::tSITAW_WAYPOINT_STR recivedWp;
                recivedWp = recivedWayPointList[i];

                WayPoint tempWp;
                tempWp.ID               = i+1;
                tempWp.wp               = Point2D(recivedWp.pose_str.coord_aF64[0], recivedWp.pose_str.coord_aF64[1]);
                tempWp.heading          = recivedWp.heading_F32;
                tempWp.velocity         = recivedWp.speed_F32;
                tempWp.entryBehaviour   = normalParking;
                tempWp.exitBehaviour    = normalParking;
                tempWp.exitDistance     = 20;

                if(recivedWp.stopConstrained_B)
                    tempWp.type             = WayPointType::stopConstrained;
                else if(recivedWp.headingConstrained_B)
                    tempWp.type             = WayPointType::headingConstrained;
                else
                    tempWp.type             = WayPointType::positionConstrained;

                    //if(planner->onRoadStatus && btStatus.openAreaStatus_B)
                       // {//DEBUG!!!!
                        wayPointList.push_back(tempWp);
                   // }
                   // else if(!planner->onRoadStatus && btStatus.openAreaStatus_B && i >= 1)
                  //      wayPointList.push_back(tempWp);


            }

            cout << "  Sending list" << endl;
            planner->setGlobalReferencePath(wayPointList);
        }
    }

     //cout << "After" << endl;

    /****************************************************************/
    /*** Check behaviour status *************************************/
    /****************************************************************/
    if(btStatus.onRoadStatus_B)
        planner->setOnRoadStatus();
    else if(btStatus.openAreaStatus_B)
        planner->setOpenAreaStatus();
    else
        planner->setIdle();
}
#endif
