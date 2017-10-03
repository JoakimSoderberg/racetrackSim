//#include "stdafx.h"
#include "ClientGame_RRT.h" 

//RRT

//Constructor
ClientGame_RRT::ClientGame_RRT(void)
{

	state_init = false;
	rrt_active=false;
	network = new ClientNetwork_RRT();
	client_type=RRT_CLIENT;

	// send init packet
	const unsigned int packet_size = sizeof(Packet_RRT_to_SERVER);
	char packet_data[packet_size];
	Packet_RRT_to_SERVER packet;
	packet.packet_type = INIT_CONNECTION;
	packet.client_type = client_type;

	packet.serialize(packet_data);

	printf("sending init packet \n");
	
	NetworkServices_RRT::sendMessage(network->ConnectSocket, packet_data, packet_size);

}

//SEND OPT.TRAJ TO SERVER
void ClientGame_RRT::send_uppdate_RRT()
{

	// send action packet
	const unsigned int packet_size = sizeof(Packet_RRT_to_SERVER);
	char packet_data[packet_size];

	packet_rrt_to_server.packet_type = RRT_EVENT;
	packet_rrt_to_server.client_type = client_type;
	

	printf("Sending trajectory to server with a size of %d bytes...  \n",packet_rrt_to_server.number_of_ref_states);

	packet_rrt_to_server.serialize(packet_data);

	NetworkServices_RRT::sendMessage(network->ConnectSocket, packet_data, packet_size);

}

State ClientGame_RRT::get_car_states(){
	return car_state_from_server;
}

list<RectObstacle> ClientGame_RRT::get_rect_obstacles(){
	return rect_obstacles_from_server;
}



void ClientGame_RRT::update(bool send_to_server)
{

	if(send_to_server && rrt_active)
		send_uppdate_RRT(); //Send uppdate bout opt.traj.

	//*********************************************************//
	//Recieving new info about car and obstacles.
	//*********************************************************//

	//I KNOW IT IS A SERVER_TO_RRT PACKET.
	Packet_SERVER_to_RRT packet;

	int data_length = network->receivePackets(network_data);

	if (data_length <= 0)
	{
		//No data recieved.
		return;
	}


		//Data has been recieved

	int i = 0;
	while (i < (int)data_length)
	{
		packet.deserialize(&(network_data[i]));

		i += sizeof(Packet_SERVER_to_RRT);

		if(packet.client_type==RRT_CLIENT){

		double x;
		double y;
		double v;
		double heading;
		double a;
		double steer_angle;
		double steer_angle_rate;

		double number_of_obstacles;

		int pos = 0;

			switch (packet.packet_type) {

			case RRT_EVENT:

				printf("client %d received RRT event packet from server\n", client_type);
				
				rrt_active=true;
				/**************************************************************************/
				/****************  Uppdate car position ***********************************/
				//*************************************************************************/

				printf("*****************************************************************\n");
				printf("RRT client recieved new car states \n");
				
				x = -packet.x_car*100;
				y = -packet.y_car*100;
				v =  packet.v_car*100;
				heading = packet.heading_car+3.14;
				//Not usning:
				a = 0;
				steer_angle = 0;
				steer_angle_rate = 0;

				printf("car_x= %f , car_y= %f , velocity= %f , heading = %f ",x,y,v,heading);
				car_state_from_server = State(x,y,v,a,heading,steer_angle,steer_angle_rate);
				printf("*****************************************************************\n");

				/**************************************************************************/
				/****************  Uppdate rectobstacle_list ******************************/
				//*************************************************************************/
				number_of_obstacles=packet.number_of_obst;
				rect_obstacles_from_server.resize(number_of_obstacles);
				
				for(list<RectObstacle>::iterator iter = rect_obstacles_from_server.begin(); iter!= rect_obstacles_from_server.end(); iter++)
				{
					double x_center = -100*packet.x_obst[pos];
					double y_center = -100*packet.y_obst[pos];
					Point2D center = Point2D(x_center,y_center);
					double width = 100*packet.width_obst[pos];
					double height = 100*packet.height_obst[pos];
					double vx = -100*packet.vx_obst[pos];
					double vy = -100*packet.vy_obst[pos];
					double heading = packet.heading_obst[pos]+3.14;

					cout << "pos of iter obstacle init: " << pos << endl;
					cout << "x: " << x_center << " y: " << y_center << endl;

					pos++;
					*iter = RectObstacle(center,width,height,vx,vy);
				}

				printf("RRT client recieve obstacle(s) \n");
				state_init=true;
				break;

			case END_RRT_EVENT:

				printf("RRT client ( %d ) received ending event packet from server\n", client_type);
				printf("wait for GO \n");
				
				rrt_active=false;
				//TELL RRT PROG TO HOLD.
				//stop(); //TODO
				break;

			default:

				printf("not packet type recieved for RRT \n");
				break;
			}
		}
	}
}


//NOT WORKING AS INTENDED
void ClientGame_RRT::stop() {

	int iResult = shutdown(network->ConnectSocket, SD_SEND);

	if (iResult == SOCKET_ERROR)
	{
		printf("shutdown failed: %d\n", WSAGetLastError());
	}

	// cleanup
	closesocket(network->ConnectSocket);
	WSACleanup();
};