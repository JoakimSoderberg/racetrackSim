#pragma once

#include <WinSock2.h>
#include <WS2tcpip.h>

using namespace std;

class NetworkServices_RRT
{

public:

	static int sendMessage(SOCKET curSocket, char * message, int messageSize);
	static int receiveMessage(SOCKET curSocket, char * buffer, int bufSize);

};