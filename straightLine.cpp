#include "straightLine.hpp"


default_random_engine generator(time(0));
uniform_real_distribution<double> distribution(0,1);

/**
 * constructor
 */

straightLine::straightLine(Eigen::MatrixXd& ob){
    qOrig.state.x = 400; 
    qOrig.state.y = -400; 
    qOrig.cost    = 0; 
    qGoal.state.x = -400; 
    qGoal.state.y = 400; 
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

void straightLine::extractPath(Path& path){
    kdNodePtr p = qGoalPtr; 
    path.cx.clear(); 
    path.cy.clear(); 
    while(p && p->parent.lock()){
        Node n1 = p->node, n2 = p->parent.lock()->node; 
        double dist = calDistNode(n1, n2), incl = dist / (2 * (dist/10));  
        double xVec = (n2.state.x - n1.state.x) / dist, yVec = (n2.state.y - n1.state.y) / dist;
        for(double i = 0; i < dist; i += incl){
            path.cx.push_back(n1.state.x + xVec * i);
            path.cy.push_back(n1.state.y + yVec * i); 
        } 
        p = p->parent.lock();
    }
    path.cx.push_back(p->node.state.x); 
    path.cy.push_back(p->node.state.y); 
}

/**
 * public
 */

void straightLine::rrtStar(Path& path){
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
                /*
                if(iter > minNodeAmount){
                    cout << "drawing..." << endl;
                    visualizer.drawMapGoalPath(tree.getRootPtr(), qGoalPtr);
                }
                */
            }
            
        }
        if(qGoalPtr != nullptr && iter > minNodeAmount){
            // visualizer.drawMap(tree.getRootPtr(), qGoal);
            extractPath(path); 
            return; 
        }            

        cout << iter << endl;
        iter++;
    }
    cout << "finish" << endl;
}

/**
 * main
 */

void smooth(vector<double>& x, vector<double>& y, vector<double>& newX, vector<double>& newY){
    double weightData = 0.1, weightSmooth = 0.5, tolerance = 0.00001; 
    newX = x; newY = y; 
    double change = tolerance; 
    while(change >= tolerance){
        change = 0.0; 
        for(int i = 1; i < x.size()-1; i++){
            double aux = newX[i], auy = newY[i]; 
            newX[i] += weightData * (x[i] - newX[i]) + weightSmooth * (newX[i-1] + newX[i+1] - 2.0 * newX[i]);
            newY[i] += weightData * (y[i] - newY[i]) + weightSmooth * (newY[i-1] + newY[i+1] - 2.0 * newY[i]);
            change  += abs(aux - newX[i]) + abs(auy - newY[i]); 
        }
    }
}

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
    Path path; 
    st.rrtStar(path); 

    std::map<std::string, std::string> keywords; 
    keywords["linewidth"] = "2"; 
    keywords["color"] = "c"; 
    plt::plot(path.cx, path.cy, keywords);

    vector<double> x, y; 
    smooth(path.cx, path.cy, x, y); 
    keywords["color"] = "k";
    plt::plot(x, y, keywords);  
    st.visualizer.drawObstacle();
    plt::show();
    
    return 0;
}
