/* Some OSAAR header 
Isak Nielsen
2011-10-19
*/

#include "refinterface.h"

using namespace std;
/* Default constructor */
RefInterface::RefInterface(unsigned int* fCount)
{
	firstRun = true;
	deltaIndex = 40;
	sumRe = 0;
	index = 0;
	frameCount = fCount;
	refFile.open(datadir+"refmeas.txt",fstream::out);
	/* Print header */
	char tmpWrite[256];
	sprintf_s(tmpWrite,"Measures from reference interface\nframeCount r_e v th_e intr_e\n");
	refFile.write((const char*)tmpWrite,strlen(tmpWrite));
	return;
}

/* Constructor that specifies which file the reference information can be read */
RefInterface::RefInterface(string fName, unsigned int* fCount)
{
	firstRun = true;
	deltaIndex = 40;
	readReferenceInfo(fName);
	sumRe = 0;
	index = 0;
	frameCount = fCount;
	refFile.open(datadir+"refmeas.txt",fstream::out);
	/* Print header */
	char tmpWrite[256];
	sprintf_s(tmpWrite,"Measures from reference interface\nframeCount r_e v th_e intr_e\n");
	refFile.write((const char*)tmpWrite,strlen(tmpWrite));
	return;
}

/* Destructor */
RefInterface::~RefInterface()
{
	for(int k = 0; k < refVec.size(); k++)
	{
		/* Deallocate all the memory */
		delete refVec.at(k);
	}
	refFile.close();
}

/* Function that reads information from the file with the reference trajectory */
void RefInterface::readReferenceInfo(string fName)
{

	RefInfo rInfo;
	char line[256];

	/* Open the file to read from */
	fstream inptFile;
	fName = datadir+fName;
	inptFile.open(fName.c_str(),fstream::in);

	if(!inptFile.is_open())
	{
		cout << "Couldn't open file " << fName << endl;
		return;
	}

	/* Read lines until eof */
	while(!inptFile.eof())
	{
		inptFile.getline(line,256);

		/* Extract information to a string */
		 //Isaks trajektoria
		/*int nrOfWritten = sscanf(line,"%f %f %f %f %f %f %f",&rInfo.x,&rInfo.y,&rInfo.v,&rInfo.th,
			&rInfo.thd,&rInfo.ug,&rInfo.us);
		if(nrOfWritten != 7)
			cout << "Wrong number of elements in reference\n";*/
		
		// 2012 ref traj

		float vx,vy,ug,us;
		int nrOfWritten = sscanf(line,"%f %f %f %f %f %f %f %f",&rInfo.x,&rInfo.y,&vx,&vy,&rInfo.th,&rInfo.thd, &ug, &us);
		if(nrOfWritten != 8)
			cout << "Wrong number of elements in reference\n";

		refVec.push_back(new RefInfo(rInfo));
	}
	
	
	
	/* WHY DO I HAVE TO DO THIS !! */
	refVec.pop_back();



	inptFile.close();
}

/* Function that calculates the reference from a given state.
	The matrix zhat is the measurements for the controller,
	vref is the velocity reference (all other are 0) and uOffline is the
	feed forward control signal calculated offline */
void RefInterface::getRef(matrix& state, matrix& zhat, float& vref, matrix& uOffline)
{
	float x = state.Get(x_index_);
	float y = state.Get(y_index_);
	float pi = 3.14159265358979323846;
	int index;

	RefInfo* rInfo;
	RefInfo* nextRInfo;

	/* Calculate the index of the reference point that is closest to the car */
	index = findIndex(state);

	/* Get the RefInfo struct containing the data */
	rInfo = refVec.at(index);
	if(index == refVec.size()-1)
		nextRInfo = refVec.at(0);
	else
		nextRInfo = refVec.at(index+1);

	/* Truncate the error in the angle to the interval -pi<the<=pi */
	float the = rInfo->th-state.Get(th_index_);

	if(the > pi)
	{
		while(the > pi)
			the -= 2*pi;
	}
	else if(the <= -pi)
	{
		while(the <= -pi)
			the += 2*pi;
	}
	/*
	the=2*pi*fmod(the,2*pi);
	if(the > pi)
		the = the - 2*pi;
//	else if(the <= -pi)
	//	the = the + 2*pi;

	/* Calculate the measurements */
	float re = calcDistError(state,rInfo,nextRInfo);
	sumRe = sumRe+0.01f*re;

	zhat.Set(1,1,re);
	zhat.Set(2,1,state.Get(v_index_));
	zhat.Set(3,1,the);
	zhat.Set(4,1,sumRe);

	/* Set the optimal control signal */
	uOffline.Set(ug_index_,1,rInfo->ug);
	uOffline.Set(us_index_,1,rInfo->us);

	/* Set speed reference */
	vref = rInfo->v;

	/* Print measurements to file */
	char tmpWrite[256];
	sprintf_s(tmpWrite,"%d %f %f %f %f\n",*frameCount, zhat.Get(1),zhat.Get(2),zhat.Get(3),zhat.Get(4));
	refFile.write((const char*)tmpWrite,strlen(tmpWrite));

	return;	
}

/* Function that searches for the index corresponding to the reference point 
	that is closest to the car */
int RefInterface::findIndex(matrix& state)
{
	int interval;
	int N = refVec.size();
	float x = state.Get(x_index_);
	float y = state.Get(y_index_);

	/* If it is the first time that the function is called, search through all
		indecies to get a global solution */
	int halfN = ceil((float)N/(float)2.0f);

	if(firstRun)
	{
		/* Set the interval to the length of the reference vector */
		interval = halfN;
		firstRun = false;
	}
	else
	{
		interval = min(deltaIndex,halfN);
	}

	int tmpInx;
	/* Initiate minDist to something large */
	float minDist = 100000;
	int minDistInx;
	float dist;
	RefInfo* rInfo;

	/* Search through the vector to find the best possible point */
	for(int s = index-interval; s < index+interval; s++)
	{
		if(s % N < 0)
		{
			/* The modulus is negative, just add N */
			tmpInx = s+N;
		}
		else
		{
			tmpInx = s % N;
		}
		
		rInfo = refVec.at(tmpInx);
		/* Since sqrt(x^2,y^2)<sqrt(x^2,z^2) -> x^2+y^2 < x^2+z^2, the sqrt never has to
			has to be calculated */
		dist = pow(x-rInfo->x,2)+pow(y-rInfo->y,2);
		if(dist < minDist)
		{
			minDist = dist;
			minDistInx = tmpInx;
		}
	}

	index = minDistInx;

	return index;
}


/* Calculates the distance error with sign */
float RefInterface::calcDistError(matrix& state, RefInfo* cur, RefInfo* next)
{
	float sign;
	float distance;
	float rx;
	float ry;
	float x;
	float y;

	/* Create the vectors rNext-rCur and x-rCur */
	rx = next->x-cur->x;
	ry = next->y-cur->y;
	x = state.Get(x_index_)-cur->x;
	y = state.Get(y_index_)-cur->y;

	/* First check the sign. A negative sign means that the point given by state is
		to the rigth of the reference trajectory */
	sign = rx*y-ry*x;
	if(sign < 0)
		sign = 1;
	else
		sign = -1;

	/* Calculate the distance with the projection formula and Phytagora's theorem */
	float projNorm = pow(rx*x+ry*y,2)/(pow(rx,2)+pow(ry,2));

	distance = sign*sqrt(max(pow(x,2)+pow(y,2)-projNorm,0.0f));
	return distance;
}
