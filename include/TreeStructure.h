#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <list>
#include <set>
#include <limits>

#include "PlannerTypes.h"

using namespace std;

typedef enum { DELETE_MARKED,
               KEEP_MARKED
             } Marking;

class TreeStructure
{

public:
//private:

    /****************************************************************/
    /*** Private varibles *******************************************/
    /****************************************************************/
    //Pointer to the root of the tree
    Node* root;

    /****************************************************************/
    /*** Private methods ********************************************/
    /****************************************************************/

    /****************************************************************/
    /*** Tree construction methods **********************************/
    /****************************************************************/
    int         insertNode(Node& vertexStartIn, Trajectory& trajectoryIn, Node& vertexEndIn);
    int         insertIntermediateNodes(Node*& parentNode, list<Point2D>& intermediateControlPoints, list<Trajectory*>	trajectoryListOut, list<Node*> addedNodesOut, Direction direction, double vRef);
    int         deleteChild(Node& nodeIn, Node& keepNode);
    void        saveRootTrajectory(Node& newRoot);
    void        removeOldNodes();

    void        markNodes(Node& keepFromNode);
    void        updateTree(Marking marking);

public:

    /****************************************************************/
    /*** Public varibles ********************************************/
    /****************************************************************/

    //list to store the tree
    list<Node*>             forwardNodeList;
    list<Node*>             reverseNodeList;
    list<Node*>             stoppingNodeList;
    priority_queue<Node*, vector<Node*>, compareLowBoundCost> stoppingNodeQueue;
    priority_queue<Node*, vector<Node*>, compareRootCost>       goalReachedQueue;

    int numNodes;
    int numForwardNodes;
    int numReverseNodes;

    /****************************************************************/
    /*** Constructor destructor *************************************/
    /****************************************************************/
    TreeStructure();
    ~TreeStructure();

    /****************************************************************/
    /*** Tree construction methods **********************************/
    /****************************************************************/
    Node*           insertNode(Node& nodeStartIn,  Sample &randomSample, Trajectory& trajectoryIn);
    int             deleteBranch(Node& nodeIn);
    void            resetTree();

    /****************************************************************/
    /*** Misc                      **********************************/
    /****************************************************************/
    void        recalculateCostToGo(WayPoint &goalPoint);
    void        propagateSafeState(Node& nodeIn);
    void        updateUpperBoundCost(Node& nodeIn);
    void        clearReachedQueue();
    void        clearStoppingQueue();

    /****************************************************************/
    /*** Get methods ************************************************/
    /****************************************************************/
    int         getLowCostSequence (list<State>& controlOut, list<Node*>& nodeSequence);
    int         getLowCostSequenceDemo2014  (list<State>& controlSequence, list<Node*>& nodeSequence);
    int         getLowCostTrajectory(list<State>& controlOut);
    Node&       getBestNode () { return *goalReachedQueue.top();}
    Node&       getRootNode () { return *root;}

    /****************************************************************/
    /*** Set methods ************************************************/
    /****************************************************************/
    void            setRootState(State state_in, double lookAheadDistance);
    void            setNewRootNode(Node& newRoot);
    void            setNewRootNodeOld(Node& newRoot);

    /****************************************************************/
    /*** Initzialisation of the tree ********************************/
    /****************************************************************/
    void initialize();
};
