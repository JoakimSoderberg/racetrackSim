#ifndef NETWORKDATA_H
#define NETWORKDATA_H

#include <string.h>

#define MAX_PACKET_SIZE 10000000

enum PacketTypes {

	INIT_CONNECTION = 0,

	RRT_EVENT = 1,

	END_RRT_EVENT = 2,

	GPGO_EVENT = 3,

	END_GPGO_EVENT = 4,
};

enum ClientTypes {

	RRT_CLIENT=0,

	GPGO_CLIENT=1,
};

struct Packet_RRT_to_SERVER {

	int packet_type;
	int client_type;

	int number_of_ref_states;
	double x_pos[600];
	double y_pos[600];
	double theta[600];


	void serialize(char * data) {
		memcpy(data, this, sizeof(Packet_RRT_to_SERVER));
	}

	void deserialize(char * data) {
		memcpy(this, data, sizeof(Packet_RRT_to_SERVER));
	}

};

struct Packet_SERVER_to_RRT {

	int packet_type;
	int client_type;

	int number_of_obst;
	double x_obst[5];
	double y_obst[5];
	double vx_obst[5];
	double vy_obst[5];
	double width_obst[5];
	double height_obst[5];
	double heading_obst[5];

	double x_car;
	double y_car;
	double heading_car;
	double v_car;

	void serialize(char * data) {
		memcpy(data, this, sizeof(Packet_SERVER_to_RRT));
	}

	void deserialize(char * data) {
		memcpy(this, data, sizeof(Packet_SERVER_to_RRT));
	}

};

struct Packet_SERVER_to_GPGO {

	int packet_type;
	int client_type;

	double laptimes[10];

	void serialize(char * data) {
		memcpy(data, this, sizeof(Packet_SERVER_to_GPGO));
	}

	void deserialize(char * data) {
		memcpy(this, data, sizeof(Packet_SERVER_to_GPGO));
	}

};

struct Packet_GPGO_to_SERVER {

	int packet_type;
	int client_type;

	double speeds[12]; //GPGO
	 

	void serialize(char * data) {
		memcpy(data, this, sizeof(Packet_GPGO_to_SERVER));
	}

	void deserialize(char * data) {
		memcpy(this, data, sizeof(Packet_GPGO_to_SERVER));
	}

};

#endif