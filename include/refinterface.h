/* Some OSAAR header
Isak Nielsen
2011-10-19
*/

#ifndef REFINTERFACE_H_
#define REFINTERFACE_H_

#include "globaldefines.h"

/** Struct for handling the reference information. Contains the reference
	state and the optimal control signal. */
typedef struct RefInfo
{
	float x;
	float y;
	float v;
	float th;
	float thd;
	float ug;
	float us;
} RefInfo;

/** Definition of the class that handles the interface between the reference 
	trajectory (from an offline calculation) and the online system s.a. the
	controller */
class RefInterface
{
public:
	/** Default constructor */
	RefInterface(unsigned int* fCount);

	/** Constructor that specifies which file the reference information can be read */
	RefInterface(std::string fName,unsigned int* fCount);

	/** Destructor */
	~RefInterface();

	/** Function that reads information from the file with the reference trajectory.
		The information must be on the form x y v th thd uh us and the points should
		be ordered */
	void readReferenceInfo(std::string fName);

	/** Function that calculates the reference from a given state.
		The matrix zhat is the measurements for the controller,
		vref is the velocity reference (all other are 0) and uOffline is the
		feed forward control signal calculated offline */
	void getRef(matrix& state, matrix& zhat, float& vref, matrix& uOffline);

	/** Function that cleares the integral */
	void clearIntegral() {sumRe = 0;}

	int getRefVecSize() { return refVec.size();}

	RefInfo* getRefInfo(int i) { return refVec[i];}
	
	/** Function that searches for the index corresponding to the reference point 
	that is closest to the car */
	int findIndex(matrix& state);

private:
	/** Calculates the distance error with sign */
	float calcDistError(matrix& state, RefInfo* cur, RefInfo* next);
	std::vector<RefInfo*> refVec;

	/** Check if it is the first run (then searched all the track) */
	bool firstRun;

	/** How many indecies before and after the last reference point should be searched */
	int deltaIndex;

	/** Used to get integral in the reference error */
	float sumRe;

	int index;

	/** Fstream object that represents the file where the measurements are saved */
	std::ofstream refFile;

	/** Pointer to the frameCounter */
	unsigned int* frameCount;


};
#endif
