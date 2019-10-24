#include "visualizer.hpp"
#include "common.hpp"
#include "kdTreeNode.hpp"

using namespace std; 

class straightLine{
private:
    kdTreeNode tree; 
    Eigen::MatrixXd obstacle;
    double width =  1000.0; 
    double height = 1000.0; 
    int iteration = 100000;

    Node qOrig; 
    Node qGoal; 
    Node qNew; 
    Node qNear; 

    

    kdNodePtr qNewPtr; 
    kdNodePtr qNearPtr; 
    kdNodePtr qGoalPtr; 
    vector<kdNodePtr> nearbyNodes; 

    void findNearbyNodes(); 
    void rewire(); 
    void randomPoint();
    void extractPath(Path& path);
public:
    Visualizer visualizer;
    
    straightLine(Eigen::MatrixXd& ob);
    void rrtStar(Path& path); 
}; 

