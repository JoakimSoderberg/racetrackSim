#include "TreeStructure.h"

/****************************************************************/
/*** Constructor destructor *************************************/
/****************************************************************/
TreeStructure::TreeStructure()
{
    root = NULL;

    numNodes		= 0;
    numForwardNodes = 0;
    numReverseNodes = 0;
}

TreeStructure::~TreeStructure()
{
    // Delete all forward nodes
    for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
        delete *iter;

    // Delete all reverse nodes
    for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
        delete *iter;

    // Delete all safe nodes
    for (list<Node*>::iterator iter = stoppingNodeList.begin(); iter != stoppingNodeList.end(); iter++)
        delete *iter;
}

/*****************************************************************/
/*** Compare methods used for sort  *****************************/
/****************************************************************/
int		compareLowerBoundCost(Node* i, Node* j) {

    return (i->getLowerBoundCost() < j->getLowerBoundCost());
}

int		compareFromRootCost(Node* i, Node* j) {

    return (i->getCostFromRoot() < j->getCostFromRoot());
}

/****************************************************************/
/*** Set methods ************************************************/
/****************************************************************/
void TreeStructure::setNewRootNode(Node& newRoot)
{
    //mark the nodes we want to keep
    markNodes(newRoot);

    //make sure we keep enough tail for the root trajectory (Important for TRIN)
    saveRootTrajectory(newRoot);

    //updates tree acordning to markings
    updateTree(KEEP_MARKED);

    newRoot.root = true;
    root = &newRoot;
}

void TreeStructure::setRootState(State state_in, double lookAheadDistance)
{
    // Initialize the root vertex
    root = new Node;
    root->setState(state_in);

    root->costFromParent = 0.0;
    root->costFromRoot = 0.0;

    Point2D stateKey;
    stateKey.x = cos(root->getState().heading)*lookAheadDistance + root->getState().x;
    stateKey.y = sin(root->getState().heading)*lookAheadDistance + root->getState().y;

    root->stateKey = stateKey;

    root->parent = NULL;
    root->root = true;
}

/****************************************************************/
/*** Get methods ************************************************/
/****************************************************************/
int TreeStructure::getLowCostSequence  (list<State>& controlSequence, list<Node*>& nodeSequence)
{

    Node* currentNode ;
    State controlState;

    nodeSequence.clear();

    if (goalReachedQueue.empty() && stoppingNodeList.empty())
    {
        return 0;
    }
    else if (goalReachedQueue.empty())
    {
        //currentNode = stoppingNodeQueue.top();
        return 0; // If path to goal was not found, do not start.
    }
    else
    {
        currentNode = goalReachedQueue.top();
    }

    while(!currentNode->isRoot())
    {
        nodeSequence.push_front(currentNode);

        controlState.x   = currentNode->getStateKey().x;
        controlState.y   = currentNode->getStateKey().y;
        //TODO fix this
        if(currentNode->getDirection() == forwardDirection)
            controlState.vx  =  currentNode->getVelocityRef();
        else
            controlState.vx  = -currentNode->getVelocityRef();

        controlSequence.push_front(controlState);

        currentNode = &currentNode->getParent();
    }

    //add the root to the control
    controlState.x   = currentNode->getStateKey().x;
    controlState.y   = currentNode->getStateKey().y;
    //TODO fix this
    if(currentNode->getDirection() == forwardDirection)
         controlState.vx  =  currentNode->getVelocityRef();
    else
         controlState.vx  =  -currentNode->getVelocityRef();

    controlSequence.push_front(controlState);

    nodeSequence.push_front(currentNode);

    return 1;

}

int TreeStructure::getLowCostSequenceDemo2014  (list<State>& controlSequence, list<Node*>& nodeSequence)
{

    Node* currentNode ;

    nodeSequence.clear();
   // cout << "Enter getSequence "<< endl;
    if (goalReachedQueue.empty() && stoppingNodeQueue.empty())
    {
       // cout << " Not feasable!"<< endl;
        return 0;
    }
    else if (goalReachedQueue.empty())
    {
       // cout << " Stopping point"<< endl;
        currentNode = stoppingNodeQueue.top();
    }
    else
    {
        //cout << " Good point"<< endl;
        currentNode = goalReachedQueue.top();
    }

   // cout << "  Before while"<< endl;
    while(!currentNode->isRoot())
    {
        nodeSequence.push_front(currentNode);

        vector<State> tempTrajectory;
        Trajectory traj;
        traj = currentNode->getTrajectory();

        list<State>::const_iterator trajectoryState = traj.stateList.begin();
        for( ; trajectoryState != traj.stateList.end(); trajectoryState++)
        {
            tempTrajectory.push_back(*trajectoryState);
        }

        for(int i = tempTrajectory.size()-1; i >= 0; i--)
        {
            controlSequence.push_front(tempTrajectory[i]);
        }


        currentNode = &currentNode->getParent();
    }
   // cout << "  After while"<< endl;

    //add the root to the control
    nodeSequence.push_front(currentNode);

    vector<State> tempTrajectory;
    list<State> stateList = currentNode->getTrajectory().stateList;

    list<State>::const_iterator trajectoryState = stateList.begin();
    for( ; trajectoryState != stateList.end(); trajectoryState++)
    {
        tempTrajectory.push_back(*trajectoryState);
    }

    for(int i = tempTrajectory.size()-1; i >= 0; i--)
    {
        controlSequence.push_front(tempTrajectory[i]);
    }

   // cout << "Return ok!"<< endl << endl;
    return 1;

}



int TreeStructure::getLowCostTrajectory(list<State>& controlOut)
{
    Node* currentNode ;
    State controlState;

    if(stoppingNodeList.size() <= 0)
        return 0;

    stoppingNodeList.sort(compareFromRootCost);
    currentNode = stoppingNodeList.front();



//REmove!! Hack for plotting
    //lowerBoundNode = currentNode;

    while(!currentNode->isRoot())
    {
        Trajectory traj;
        traj = currentNode->getTrajectory();

        list<State>::const_iterator trajectoryState;
        for(trajectoryState = traj.stateList.end(); trajectoryState != traj.stateList.begin(); trajectoryState--)
        {
            controlOut.push_front(*trajectoryState);
        }

        currentNode = &currentNode->getParent();
    }

    //Add root trajectory
    Trajectory traj;
    traj = currentNode->getTrajectory();

    list<State>::const_iterator trajectoryState;
    for(trajectoryState = traj.stateList.begin(); trajectoryState != traj.stateList.end(); trajectoryState++)
    {
         controlOut.push_front(*trajectoryState);
    }

    return 1;
}

/****************************************************************/
/*** Misc                      **********************************/
/****************************************************************/
void TreeStructure::propagateSafeState(Node& nodeIn)
{
    Node* currentNode = &nodeIn;
    while(!currentNode->isRoot() &&  !currentNode->isSafe())
    {
        currentNode->setSafe();

        currentNode = &currentNode->getParent();

    }
}

void TreeStructure::updateUpperBoundCost (Node& nodeIn)
{
    Node* currentNode = &nodeIn;

    while(!currentNode->isRoot() &&  currentNode->getParent().upperBoundCost > currentNode->upperBoundCost + currentNode->costFromParent)
    {
        currentNode->getParent().upperBoundCost = currentNode->upperBoundCost + currentNode->costFromParent;

        currentNode = &currentNode->getParent();
    }
}

void TreeStructure::clearStoppingQueue()
{
    //.clear not implemented for priority queue. Clear by creating a new empty queue. Elements in old queue is already deleted.
    stoppingNodeQueue = priority_queue<Node*, vector<Node*>, compareLowBoundCost>();
}

void TreeStructure::clearReachedQueue()
{
    //.clear not implemented for priority queue. Clear by creating a new empty queue. Elements in old queue is already deleted.
    goalReachedQueue = priority_queue<Node*, vector<Node*>, compareRootCost>();
}

/****************************************************************/
/*** Tree construction methods **********************************/
/****************************************************************/
Node* TreeStructure::insertNode(Node& nodeStartIn, Sample &randomSample, Trajectory& trajectoryIn)
{

    // Create new node
    Node* nodeNew		= new Node;
    nodeNew->stateKey	= randomSample.samplePoint;
    nodeNew->parent		= NULL;
    nodeNew->getState() = trajectoryIn.getEndState();
    nodeNew->direction	= randomSample.direction;
    nodeNew->vRef		= randomSample.velocity;

    if(randomSample.direction == Direction::forwardDirection)
    {
        this->forwardNodeList.push_front (nodeNew);
        this->numForwardNodes++;
    }
    else if(randomSample.direction == Direction::reverseDirection)
    {
        this->reverseNodeList.push_front (nodeNew);
        this->numReverseNodes++;
    }

    this->numNodes++;

    // Insert the trajectory between the start and end vertices
    insertNode (nodeStartIn, trajectoryIn, *nodeNew);

    return nodeNew;
}

int TreeStructure::insertNode(Node& nodeStartIn, Trajectory& trajectoryIn, Node& nodeEndIn) {

    // Update the costs
    nodeEndIn.costFromParent = trajectoryIn.evaluateCost();
    nodeEndIn.costFromRoot = nodeStartIn.costFromRoot + nodeEndIn.costFromParent;

    nodeEndIn.trajFromParent = trajectoryIn;

    // Update the parent to the end vertex
    if (nodeEndIn.parent)
        nodeEndIn.parent->children.erase(&nodeEndIn);
    nodeEndIn.parent = &nodeStartIn;

    // Add the end vertex to the set of children
    nodeStartIn.children.insert(&nodeEndIn);

    return 1;
}

int TreeStructure::deleteBranch(Node& nodeIn)
{
    if(!nodeIn.isRoot())
    {
        //Mark the branch
        markNodes(nodeIn);


        nodeIn.parent->children.erase(&nodeIn);

        //Delete marked nodes
        updateTree(DELETE_MARKED);

		return 1; //Added OL VS was whining (Must return a value) don't think this fnk is used?
    }

	return 0; //Added OL VS was whining (Must return a value) don't think this fnk is used?
}

void TreeStructure::updateTree(Marking marking)
{
    //cout << "Entering newRoot" << endl;

    list<Node*> tempForwardNodeList;
    list<Node*> tempReverseNodeList;
    list<Node*> tempStoppingNodeList;
    priority_queue<Node*, vector<Node*>, compareRootCost>       tempGoalReachedQueue;
    priority_queue<Node*, vector<Node*>, compareLowBoundCost>   tempStoppingNodeQueue;

    bool markingKeep;

    if(marking == DELETE_MARKED)
        markingKeep = false;
    else if(marking == KEEP_MARKED)
        markingKeep = true;

    for (list<Node*>::iterator iter = stoppingNodeList.begin(); iter != stoppingNodeList.end(); iter++)
    {
        Node* node = *iter;
        if(node->marked == markingKeep)
        {
            tempStoppingNodeList.push_back(node);
            tempStoppingNodeQueue.push(node);
        }
    }

   while(!goalReachedQueue.empty())
    {
        Node* node = goalReachedQueue.top();
        goalReachedQueue.pop();

        if(node->marked == markingKeep)
        {
            tempGoalReachedQueue.push(node);
        }
    }

    for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
    {
        Node* node = *iter;
        if(node->marked != markingKeep)
        {
            delete node;
            numNodes--;
            numForwardNodes--;
        }
        else
        {
            node->marked = false;
            tempForwardNodeList.push_back(node);
        }
    }

   for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
    {
        Node* node = *iter;
        if(node->marked != markingKeep)
        {
            delete node;
            numNodes--;
            numReverseNodes--;
        }
        else
        {
            node->marked = false;
            tempReverseNodeList.push_back(node);
        }
    }


   forwardNodeList.clear();
   reverseNodeList.clear();
   stoppingNodeList.clear();
   clearReachedQueue();
   clearStoppingQueue();

   forwardNodeList      = tempForwardNodeList;
   reverseNodeList      = tempReverseNodeList;
   stoppingNodeList     = tempStoppingNodeList;
   goalReachedQueue     = tempGoalReachedQueue;
   stoppingNodeQueue    = tempStoppingNodeQueue;

  // cout << "Exit newRoot" << endl;

}

void TreeStructure::markNodes(Node& keepFromNode)
{
    set<Node*>::iterator n;
    for(n=keepFromNode.children.begin(); n!=keepFromNode.children.end(); n++)
    {
        Node* child = *n;

        markNodes(*child);

        child->marked = true;
    }

    if(!keepFromNode.isRoot())
       keepFromNode.marked = true;
}

void TreeStructure::saveRootTrajectory(Node& newRoot)
{
//    FILE * pFile;

//    pFile = fopen ("debug.txt","w");

    if(newRoot.parent != NULL)
    {
        Node* currentNode = newRoot.parent;

        list<State>::reverse_iterator trajectoryState = currentNode->trajFromParent.stateList.rbegin();
        for( ; trajectoryState != currentNode->trajFromParent.stateList.rend(); trajectoryState++)
        {
            if(newRoot.trajFromParent.stateList.size() < 50)
                newRoot.trajFromParent.stateList.push_front((*trajectoryState));
        }

    }
}

void TreeStructure::resetTree()
{
    Node* rootBackup = NULL;
    if (root)
    {
        //root->children.clear();
        //root->trajFromParent = NULL;
        rootBackup = new Node(*root);
        rootBackup->children.clear();
    }

    // Delete all the vertices
    for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
        delete *iter;

    // Delete all the vertices
    for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
        delete *iter;

    forwardNodeList.clear();
    stoppingNodeList.clear();
    reverseNodeList.clear();
    clearReachedQueue();
    clearStoppingQueue();

    numForwardNodes = 0;
    numReverseNodes = 0;
    numNodes = 0;
//    lowerBoundCost = numeric_limits<double>::max();
//    lowerBoundNode = NULL;

    if(rootBackup->getDirection() == forwardDirection)
    {
        forwardNodeList.push_back(rootBackup);
        numForwardNodes++;
        numNodes++;
    }
    else
    {
        reverseNodeList.push_back(rootBackup);
        numReverseNodes++;
        numNodes++;
    }

    root = rootBackup;
}

/****************************************************************/
/*** Initzialisation of the tree ********************************/
/****************************************************************/
void TreeStructure::initialize()
{
    // Backup the root
    Node* rootBackup = NULL;
    if (root)
        rootBackup = new Node(*root);

    // Delete all the vertices
    for (list<Node*>::iterator iter = forwardNodeList.begin(); iter != forwardNodeList.end(); iter++)
        delete *iter;

    // Delete all the vertices
    for (list<Node*>::iterator iter = reverseNodeList.begin(); iter != reverseNodeList.end(); iter++)
        delete *iter;

    forwardNodeList.clear();
    stoppingNodeList.clear();
    reverseNodeList.clear();
    clearReachedQueue();
    clearStoppingQueue();

    numForwardNodes = 0;
    numReverseNodes = 0;
    numNodes = 0;
//    lowerBoundCost = numeric_limits<double>::max();
//    lowerBoundNode = NULL;


    // Initialize the variables
    root = rootBackup;

    if (root)
    {
        root->children.clear();
        forwardNodeList.push_back(root);
        numForwardNodes++;
        numNodes++;
    }
}
