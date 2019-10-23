#include "straightLine.hpp"


default_random_engine generator(time(0));
uniform_real_distribution<double> distribution(0,1);

/**
 * constructor
 */

straightLine::straightLine(Eigen::MatrixXd& ob){
    qOrig.state.x = 400; 
    qOrig.state.y = 300; 
    qOrig.cost    = 0; 
    qGoal.state.x = -400; 
    qGoal.state.y = -400; 
    qGoal.cost    = 0; 

    tree.treeInit(qOrig);

    obstacle = ob;

    visualizer.forDebugging(obstacle);
}

/**
 * private
 */

void straightLine::randomPoint(){
    qNew.state.x  = width*distribution(generator)+(0-width/2);
    qNew.state.y  = height*distribution(generator)+(0-height/2);
    qNew.cost     = 0;
}

void straightLine::findNearbyNodes(){
    nearbyNodes = tree.Nearby(qNew, 150.0); 
}

void straightLine::rewire(){
    int n = nearbyNodes.size(); 
    for(int i = 0; i < n; i++){
        Node n = nearbyNodes[i]->node; 
        if(CollisionCheck(qNew, n, obstacle)){
            double tempCost = calDistNode(qNew, n) + qNew.cost; 
            if(tempCost < n.cost){
                nearbyNodes[i]->parent    = qNewPtr; 
                nearbyNodes[i]->node.cost =  tempCost; 
            }
        }
    }
}

/**
 * public
 */

void straightLine::rrtStar(){
    int iter = 0, minNodeAmount = 0; 
    cout << "Input minimum amount of nodes: " << endl; 
    cin >> minNodeAmount; 

    while(iter < iteration){
        randomPoint(); 
        qNearPtr = tree.findNearestPtr(qNew);
        qNear    = qNearPtr->node; 
        if(CollisionCheck(qNew, qNear, obstacle) && calDistNode(qNew, qNear) < 150.0){
            qNew.cost = calDistNode(qNew, qNear) + qNear.cost; 
            qNewPtr = tree.insert(qNew); 
            qNewPtr->parent = qNearPtr;
            findNearbyNodes(); 
            rewire(); 
            
            if(qGoalPtr != nullptr || calDistNode(qNew, qGoal) < 20.0){
                // goal part
                if(qGoalPtr == nullptr){
                    qGoalPtr = tree.insert(qGoal); 
                    qGoalPtr->parent = qNewPtr; 
                    qGoalPtr->node.cost = calDistNode(qGoal, qNew) + qNewPtr->node.cost; 
                }else if(calDistNode(qNew, qGoal) < 20.0){
                    if(calDistNode(qGoal, qNew) + qNewPtr->node.cost < qGoalPtr->node.cost){
                        qGoalPtr->parent = qNewPtr; 
                        qGoalPtr->node.cost = calDistNode(qGoal, qNew) + qNewPtr->node.cost;
                    }
                }
                if(iter > minNodeAmount){
                    cout << "drawing..." << endl;
                    visualizer.drawMapGoalPath(tree.getRootPtr(), qGoalPtr);
                }
            }
            
        }
        if(qGoalPtr == nullptr && iter > minNodeAmount)
            visualizer.drawMap(tree.getRootPtr(), qGoal);            

        cout << iter << endl;
        iter++;
    }
    cout << "finish" << endl;
}

/**
 * main
 */

int main(){
    // Eigen::MatrixXd obstacle(4,4);
    // obstacle.row(0) << 1,1,-1,1;
    // obstacle.row(1) << -1,1,-1,-1;
    // obstacle.row(2) << -1,-1,1,-1;
    // obstacle.row(3) << 1,-1,1,1;
    // obstacle = 100*obstacle;

    Eigen::MatrixXd obstacle(5,4);
    obstacle.row(0) << 50,0,50,150;
    obstacle.row(1) << 150,200,150,100;
    obstacle.row(2) << 150,100,100,100;
    obstacle.row(3) << 125,0,125,50;
    obstacle.row(4) << 200,75,150,75;
    obstacle = obstacle-(100*Eigen::MatrixXd::Ones(5,4));

    obstacle.col(0) = -1*obstacle.col(0);
    obstacle.col(2) = -1*obstacle.col(2);
    obstacle *= 5; 

    straightLine st(obstacle); 
    st.rrtStar(); 
    return 0;
}