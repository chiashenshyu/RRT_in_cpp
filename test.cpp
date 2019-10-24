#include "common.hpp"

void smooth(vector<double>& x, vector<double>& y, vector<double>& newX, vector<double>& newY){
    double weightData = 0.1, weightSmooth = 0.1, tolerance = 0.00001; 
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
        cout << change << endl;
    }
}

int main(){
    vector<double> x = {0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 4.0};
    vector<double> y = {0.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 3.0, 4.0}; 
    vector<double> newX, newY; 
    smooth(x, y, newX, newY);

    plt::plot(x, y, "r-");
    plt::plot(newX, newY, "b-");

    plt::xlim(-1, 5);
    plt::ylim(-1, 5);
    plt::show();
}