#include "TrajectoryInspector.h"


//void process_line(const std::string line);
bool isFloat(const std::string &num);

TrajectoryInspector::TrajectoryInspector(int jointNum)
{
    jointNo = jointNum;
    //This tells us how many columns to look at
    //Define the size of the outer vector
}

void TrajectoryInspector::readCSV(const std::string &filename)
{
    //Need to find the number of rows(resolution of the trajectory)
    rapidcsv::Document doc(filename, rapidcsv::LabelParams(0,0),rapidcsv::SeparatorParams(),rapidcsv::ConverterParams(true));
    std::vector<std::string> rownames =  doc.GetRowNames();
    //Identify the last row
    std::cout << "The last Row is " << rownames.back() << std::endl;
    //Sort these items into vectors
    int pathLength =  std::stoi(rownames.back());
    csvData = std::vector<Eigen::VectorXd>(pathLength + 1);
    //Construct the rows
    for(int i = 1; i <= pathLength; i++)
    {
        Eigen::VectorXd row(jointNo);
        //Construct the row
        std::vector<std::string> rowNames = doc.GetRowNames();
        bool rowExists = (std::find(rowNames.begin(), rowNames.end(),std::to_string(i)) != rowNames.end());
        if(!rowExists)
        {
            
            //Set the row to Nans
            row << nan(""),nan(""),nan(""),nan("");
            //row(1) = nan("");
            //row(2) = nan("");
            //row(3) = nan("");
            continue;
        }
        //How to deal with 
        double j1 = doc.GetCell<double>("Joint1", std::to_string(i).c_str());
        double j2 = doc.GetCell<double>("Joint2", std::to_string(i).c_str());
        double j3 = doc.GetCell<double>("Joint3", std::to_string(i).c_str());
        double j4 = doc.GetCell<double>("Joint4", std::to_string(i).c_str());
        row << j1,j2,j3,j4;
        std::cout << std::to_string(i) << ": "<<std::to_string(j1) << std::endl;
        csvData.push_back(row);
    }
    //std::cout << std::to_string(csvData)
    //printf("Here\n");

    return;
}

void TrajectoryInspector::process_line(const std::string &line, const int lineNo)
{
    //Store the elements of the string in an arrray for later storage into the struct
    std::string strElement;

    std::cout<< "Line: " <<line<<std::endl;
    float cell;
    int columns = (*this).jointNo + 1; //Needs to account for the index num as well
    std::vector<std::string> strings;
    std::vector<float> row;
    std::istringstream in(line.c_str()); //Getting some dodgy stuff here, null chars
    while(std::getline(in, strElement, ','))
    {
        std::cout << " here: " << strElement;
        if(!isFloat(strElement))
        {
            exit(1);
            return;
        }
        cell = std::stof(strElement, NULL);
        row.push_back(cell);
    } 
    std::cout<< "row:";
    for (int i; i < row.size(); i++){
        std::cout << std::to_string(row.at(i)) << " ";
    }
    std::cout <<std::endl;
    return;
}

bool isFloat(const std::string &num)
{
    //checks if the string is a valid float
    bool decimalFound = false;
    bool negativeFound = false;
    for (std::string::size_type i = 0; i < num.size(); i++)
    {
        std::cout << num[i] << std::endl;

        if(!isdigit(num[i]))
        {
            //check if the character is a decimal, only one is allowed:
            if(num[i] == '.')
            {
                if(decimalFound){
                    return false;
                }
                decimalFound = true;
            }
            else if (num[i] == '-')
            {
                if (i != 0)
                {   
                    return false;
                }
            }
        }

    }
    return true;
} 
/*
Eigen::VectorXd TrajectoryInspector::getJointAngles (float32 index)
{
    return;
}*/