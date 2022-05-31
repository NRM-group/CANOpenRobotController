#ifndef LOOKUP_TABLE_H
#define LOOKUP_TABLE_H
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include "rapidcsv.h"

#define LINEAR      1
#define DISCRETE_POINTS    100
class LookupTable
{
private:
    //TODO:
    //Figure out how to store the traj values
    //std::vector indexes;
    int jointNo;
    int pointNo; //Defines the rows assigned to the csvData
    Eigen::MatrixXd csvData; //Matrix that stores the data struture of the csv
    enum trajMode { Linear = 1, SCurve  = 2};
    int mode = LINEAR; //Interpolation mode
public:
    LookupTable(int jointNum = 4);
    void readCSV(const std::string &filename ); //String inspector constructor reads CSV of points
    void interpolateTrajectories(void);
    //Interpolate between the points.
    void interpolatePoints(std::vector<double> values, std::vector<int>rows,int col); 
    double getPosition(int jointNo, double index); //Returns the position of the gait given joint No and gait location
    int nanBelow(int row, int col);
    
    Eigen::VectorXd getJointPositions(int gaitIndex); //Returns the joint positions in incremental joint No
};


#endif