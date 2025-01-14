#include "common.hpp"
#include "ParticleFilter.hpp"
#include "planner.hpp"
#include "ppc.hpp"

#define VIZ

void smooth(std::vector<double>& x, std::vector<double>& y, std::vector<double>& newX, std::vector<double>& newY){
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

int main()
{
    /**
     * RRT star planner
     */
    planner_params A;
    A.origin = Point( 400, -400);
    A.goal   = Point(-400,  400);
    MatrixXd obstacle(5,4);

    // Simple Rectangle obstacle
    // obstacle.row(0) << 1,1,-1,1;
    // obstacle.row(1) << -1,1,-1,-1;
    // obstacle.row(2) << -1,-1,1,-1;
    // obstacle.row(3) << 1,-1,1,1;
    // A.obstacle = 100*obstacle;

    // Maze Map
    obstacle.row(0) << 50,0,50,150;
    obstacle.row(1) << 150,200,150,100;
    obstacle.row(2) << 150,100,100,100;
    obstacle.row(3) << 125,0,125,50;
    obstacle.row(4) << 200,75,150,75;
    A.obstacle = obstacle-(100*MatrixXd::Ones(5,4));

    A.obstacle.col(0) = -1*A.obstacle.col(0);
    A.obstacle.col(2) = -1*A.obstacle.col(2);
    A.obstacle *= 5; 

    A.iterations = 40000;
    A.width      = 1000; 
    A.height     = 1000;
    A.goalProx   = 15;

    Planner p(A);
    
    if(p.RRTstar()){
        cout << "Could not find path to goal" << endl;
        return 1;
    }
    
    Path path; 
    std::vector<Node> wayPoints; 
    p.ExtractPath(path, wayPoints);
    reverse(path.cx.begin(), path.cx.end()); 
    reverse(path.cy.begin(), path.cy.end());
    std::vector<double> _x, _y; 
    smooth(path.cx, path.cy, _x, _y);
    path.cx = _x; 
    path.cy = _y;

    /** 
     * Visualizer
     */
    #ifdef VIZ
    Visualizer viz; 
    viz.plannerParamsIn(A);
    #endif

    /**
     * PF Localization
     */
    ParticleFilter pf(1000);  
    std::vector<std::vector<double>> landmark = {{   0,  400},
                                                 {-450,  450},
                                                 {-400, -400},
                                                 { 100, -300},
                                                 { 500,  350},
                                                 { 425, -450}
                                                 };
    std::vector<bool> lmVisible(landmark.size(), false);
 
    /** 
     * Pure Pursuit Controller
     */
    double targetSpeed = 15;
    // std::cout << "Enter target speed between 5 and 30: " << std::endl;
    // std::cin >> targetSpeed; 
    // while(true){
    //     if(std::cin.fail() || targetSpeed < 5.0 || targetSpeed > 30.0){
    //         std::cin.clear(); 
    //         std::cin.ignore(std::numeric_limits<streamsize>::max(), '\n');
    //         std::cout << "Enter target speed between 5 and 30: " << std::endl;
    //         std::cin >> targetSpeed;
    //     }else if(!std::cin.fail()){
    //         break;
    //     }
    // }

    ppc car(A.origin.x, A.origin.y, -M_PI, 0.0); 
    ppc carP(A.origin.x, A.origin.y, -M_PI, 0.0);
    int lastIndex = path.cx.size()-1, currentIndex = 0, currentIndexP = 0; 
    // double mTime = 0.0; 
    std::vector<double> x = {car.st.x}, xp = x;
    std::vector<double> y = {car.st.y}, yp = y;
    std::vector<double> theta = {car.st.theta}, thetap = theta;
    std::vector<double> v = {car.st.v}, vp = v;
    // std::vector<double> t = {mTime};

    bool init = false; 
    while(lastIndex > currentIndex && currentIndexP < lastIndex){
        if(!init){
            init = true; 
            std::vector<double> nullP; 
            pf.priorUpdate(car.st, nullP); 
        }
        
        std::vector<double> ret = car.implementPPC(path, targetSpeed, currentIndex);
        std::vector<double> retP = carP.implementPPC(path, targetSpeed, currentIndex);
        currentIndex = ret[0];
        currentIndexP = retP[0];
        
        for(int i = 0; i < landmark.size(); i++){
            Point p1(car.st.x, car.st.y), p2(landmark[i][0], landmark[i][1]); 
            lmVisible[i] = CollisionCheckPoint(p1, p2, A.obstacle)? true : false;
        }

        std::vector<double> param(ret.begin()+1, ret.begin()+5);
        pf.priorUpdate(car.st, param); 
        pf.observation(car.st, landmark, lmVisible);
        pf.assignWeightLandmark();
        pf.resample();
        double xAvg, yAvg;  
        pf.calAverage(xAvg, yAvg); 
        cout << "difference-> x: " << abs(xAvg - carP.st.x);
        cout << ", y: " << abs(yAvg - carP.st.y) << endl;
        car.st.x = xAvg; car.st.y = yAvg; 

        x.push_back(car.st.x); 
        y.push_back(car.st.y); 
        v.push_back(car.st.theta); 
        theta.push_back(car.st.theta);
        xp.push_back(carP.st.x); 
        yp.push_back(carP.st.y); 
        vp.push_back(carP.st.theta); 
        thetap.push_back(carP.st.theta);

        #ifdef VIZ
        plt::clf(); 
        // plt::figure_size(1200, 780); 
        // plt::xlim(-10, 60); 
        // plt::ylim(-25, 25); 
        for(auto& n : wayPoints){
            plotPoint(n.state.x, n.state.y, "ro");
        }
        plt::named_plot("path", path.cx, path.cy, "k-");
        plt::named_plot("PF_Localization", x,  y,  "g-");
        plt::named_plot("GT", xp, yp, "r-");
        plotCar(carP.st.x, carP.st.y, carP.st.theta);
        plotCar(car.st.x, car.st.y, car.st.theta, "g");
        for(int i = 0; i < landmark.size(); i++){
            plotPoint(landmark[i][0], landmark[i][1], "c*"); 
            if(!lmVisible[i]) continue;
            plotLine(landmark[i][0], landmark[i][1], car.st.x, car.st.y, "c-");
        }
        viz.drawObstacle();
        // plt::named_plot("Traj_woPF", xWoLoc, yWoLoc, "c*");

        // plt::named_plot("pfTraj", pfX, pfY, "co");
        plt::title("PurePursuitControl"); 
        plt::legend(); 
        plt::pause(0.001);
        plt::xlim(-A.width/2-50, A.width/2+50);
        plt::ylim(-A.height/2-50, A.height/2+50);
        // plt::axis("equal");
        #endif
    }
    plt::show();
    return 0;
}
