#include "LookupTable.h"


//void process_line(const std::string line);
bool isFloat(const std::string &num);

LookupTable::LookupTable(int jointNum)
{
    jointNo = jointNum;
    //This tells us how many columns to look at
    //Define the size of the outer vector
}

/**
 * @brief Reads the CSV and appends it to the matrix row. Assumes that at leastthe 
 * first and last index of the CSV is defined. If the header is not defined, the 
 * program will interpolate lineraly
 * 
 * @param filename 
 */
void LookupTable::readCSV(const std::string &filename)
{
    //Need to find the number of rows(resolution of the trajectory)
    rapidcsv::Document doc(filename, rapidcsv::LabelParams(0,0),rapidcsv::SeparatorParams(),rapidcsv::ConverterParams(true));
    std::vector<std::string> rownames =  doc.GetRowNames();
    //Identify the last row
    std::cout << "The last Row is " << rownames.back() << std::endl;
    //Sort these items into vectors
    int pathLength =  std::stoi(rownames.back());
    csvData = Eigen::MatrixXd (pathLength, jointNo);
    //Construct the rows
    for(int i = 1; i <= pathLength; i++)
    {
        //Construct the row
        std::vector<std::string> rowNames = doc.GetRowNames();
        bool rowExists = (std::find(rowNames.begin(), rowNames.end(),std::to_string(i)) != rowNames.end());
        if(!rowExists)
        {
            for (int j = 0; j< jointNo; j++)
            {
                csvData(i-1,j) = nan("");
            }
            continue;
        } else 
        {
            for (int j = 1; j <= jointNo; j ++)
            {
                std::string numStr = std::to_string(j);
                std::string baseName = "Joint";
                const char* baseStr = baseName.c_str();
                const char* numChar = numStr.c_str();
                std::string jointName = std::strcat((char*)baseStr, numChar);

                double joint = doc.GetCell<double>(jointName.c_str(),std::to_string(i).c_str());
                csvData(i-1,j-1) = joint;
            }
        }
    }
    LookupTable::interpolateTrajectories(Linear);
    return;
}

void LookupTable::interpolateTrajectories(int mode) {
    //For each "Nan" item,  identify the interpolation depending on the mode
    //Going by column is better
    std::cout << LookupTable::csvData(20,2) << std::endl;

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
Eigen::VectorXd LookupTable::getJointAngles (float32 index)
{
    return;
}*/