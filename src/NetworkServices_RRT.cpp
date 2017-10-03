//#include "stdafx.h"  //Needed for Visual studio.
#include "NetworkServices_RRT.h"

int NetworkServices_RRT::sendMessage(SOCKET curSocket, char * message, int messageSize)
{
	return send(curSocket, message, messageSize, 0);
}

int NetworkServices_RRT::receiveMessage(SOCKET curSocket, char * buffer, int bufSize)
{
	return recv(curSocket, buffer, bufSize, 0);
}