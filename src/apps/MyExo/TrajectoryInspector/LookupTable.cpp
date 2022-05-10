#include "LookupTable.h"


//void process_line(const std::string line);
bool isFloat(const std::string &num);
void printMatrix(Eigen::MatrixXd matrix, int rows, int cols);

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
    std::vector<std::string> rowNames = doc.GetRowNames();
    //Identify the last row
    std::cout << "The last Row is " << rowNames.back() << std::endl;
    //Sort these items into vectors
    int pathLength =  std::stoi(rowNames.back());
    printf("Pathelength: %d\n", pathLength);
    printf("Joint No %d\n", jointNo);
    //Assign the row num
    pointNo = pathLength;
    csvData = Eigen::MatrixXd (pathLength, jointNo);
    //Construct the rows
    for(int i = 1; i <= pathLength; i++)
    {
        //Construct the row
        
        bool rowExists = (std::find(rowNames.begin(), rowNames.end(),std::to_string(i)) != rowNames.end());
        if(!rowExists)
        {
            for (int j = 0; j< jointNo; j++)
            {
                csvData(i-1,j) = nan("");
            }
            continue;
        }
        else 
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
    printf("Original Matrix:\n");
    printMatrix(csvData,  pointNo, jointNo);
    LookupTable::interpolateTrajectories(Linear);
    printf("Next Matrix:\n");
    printMatrix(csvData, pointNo, jointNo);
    return;
}


//Interpolate the unkown trajectories, there will be a few use cases
void LookupTable::interpolateTrajectories(int mode) {
    //For each "Nan" item,  identify the interpolation depending on the mode
    //Going by column is better
    std::cout <<csvData(20,3) << std::endl;
    
    for(int j = 0; j < jointNo; j++) {
        //The first value must  be defined, if it is not, flag an error
        if(csvData(0, j) == nan("")) {
            throw std::invalid_argument("trajectory start point must be defined");
        }
            
        for (int i = 0; i <(pointNo - 1); i ++) {
            //Investigate each row (Point) for each joint
            if(nanBelow(i, j)) {
                printf("No definition found for <%d> <%d>\n\r", i, j);
                //Identify the end points associated with the series of NANs
                double startPoint = csvData(i,j);
                int endRow = i+1;
                while(csvData(endRow,j) != csvData(endRow, j)) {
                    endRow++;
                }
                //The row wil be the endpoint of the interpolation
                double endPoint = csvData(endRow, j);
                std::vector<double> values {startPoint, endPoint};
                std::vector<int> rows{i, endRow};
                LookupTable::interpolatePoints(values, rows, j, LINEAR);
                i = endRow - 1;
            }
        }
    }
    return;
}

//Returns true in the event that there is a Nan below the element specified by 
//row and col. Returns 1 for the a Nan below (Equivalent to the proceed)
int LookupTable::nanBelow(int row, int col) {
    double nextValue = csvData(row+1, col);
    if (nextValue != nextValue) {
        return 1;
    }
    return 0;
}

//Interpolates between the two points and fills in the csvData
void LookupTable::interpolatePoints(std::vector<double> values, std::vector<int>rows, int col,int mode) {
    double startVal = values[0];
    double endVal = values[1];
    int startRow = rows[0];
    int endRow = rows[1];
    printf("VAL: %f --> %f\n", startVal, endVal);
    printf("ROW: %d --> %d\n", startRow, endRow);



    if(mode == LINEAR) {
        double m = (endVal - startVal) / (endRow - startRow);
        //Identify the points 
        double c = startVal - m * startRow;
        //Compute the next values in the column
        for (int i = startRow; i <endRow; i++) {
            csvData(i, col) = m * i + c;
        }
    }

}

void printMatrix(Eigen::MatrixXd matrix, int rows, int cols) {
    for (int i = 0; i < rows; i ++) {
        for (int j = 0; j < cols; j ++) {
            printf("%f ", matrix(i,j));
        }
        printf("\n");
    }
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
