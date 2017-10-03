#include <winsock2.h>
#include <Windows.h>
#include "ClientNetwork_RRT.h"
//For state
#include "PlannerTypes.h"
#include "ObstacleMap.h"


class ClientGame_RRT
{

public:

	ClientGame_RRT();
	~ClientGame_RRT(void);

	ClientNetwork_RRT* network;

	//void sendActionPackets();
	
	void send_uppdate_RRT();

	char network_data[MAX_PACKET_SIZE];

	//BOOL SETS BY PROGRAM.
	//DO UPPDATE IN MAIN-LOOP
	void update(bool send_to_server);

	void stop();

	//Packet to send to server
	Packet_RRT_to_SERVER packet_rrt_to_server;

	State get_car_states();

	list<RectObstacle> get_rect_obstacles();

	bool state_of_car_and_obst_init(){return state_init;}
	

	
private:
	
	list<RectObstacle> rect_obstacles_from_server;
	State car_state_from_server;

	bool state_init;

	bool rrt_active;

	ClientTypes client_type;
};