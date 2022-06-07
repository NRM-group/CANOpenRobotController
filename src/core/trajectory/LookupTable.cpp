#include "LookupTable.h"


//void process_line(const std::string line);

LookupTable::LookupTable(int jointNum)
{
    jointNo = jointNum;

}

/**
 * @brief Reads the CSV and appends it to the matrix row. Assumes that at leastthe 
 * first and last index of the CSV is defined. If the header is not defined, method 
 * will interpolate the points linearly
 * 
 * 
 * @param filename 
 */
void LookupTable::readCSV(const std::string &filename)
{
    /* Read a gait cycle defined by a csv file. Refer to the X2 Template for an example.
    Inputs:
        filename: String of the file to be read, note that in ROS this needs to be an absolute file path
    */
    std::vector<std::string> rowNames;
    rapidcsv::Document doc;
    try
    {
        doc = rapidcsv::Document(filename, rapidcsv::LabelParams(0,0),rapidcsv::SeparatorParams(),rapidcsv::ConverterParams(true));
        rowNames = doc.GetRowNames();
        //Identify the last row
    }
    catch( const std::ios_base::failure & e)  {
        throw(std::invalid_argument("File Not found"));
        exit(1);
    
    }

    //Sort these items into vectors
    int pathLength =  std::stoi(rowNames.back()) + 1;
    //Assign the row num

    pointNo = pathLength; //Number of discrete points per joint 
    csvData = Eigen::MatrixXd (pathLength + 1, jointNo);
    //Construct the rows
    for(int i = 0; i < pathLength; i++)
    {
        //Construct the row
        
        bool rowExists = (std::find(rowNames.begin(), rowNames.end(),std::to_string(i)) != rowNames.end());
        if(!rowExists)
        {
            for (int j = 0; j< jointNo; j++)
            {
                csvData(i,j) = nan("");
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
                csvData(i,j-1) = joint;
            }
        }
    }
    LookupTable::interpolateTrajectories();
    return;
}


//Interpolate the unkown trajectories, there will be a few use cases
void LookupTable::interpolateTrajectories(void) {
    //For each "Nan" item,  identify the interpolation depending on the mode
    //Going by column is better
    
    for(int j = 0; j < jointNo; j++) {
        //The first value must  be defined, if it is not, flag an error
        if(csvData(0, j) == nan("")) {
            throw std::invalid_argument("trajectory start point must be defined");
        }
            
        for (int i = 0; i <pointNo; i ++) {
            //Investigate each row (Point) for each joint
            if(nanBelow(i, j)) {
                //Identify the end points associated with the series of NANs
                double startPoint = csvData(i,j);
                // FIXME: error here when checking the last row - going to have a memory error
                int endRow = i+1;
                while(csvData(endRow,j) != csvData(endRow, j)) {
                    endRow++;
                }
                //The row wil be the endpoint of the interpolation
                double endPoint = csvData(endRow, j);
                std::vector<double> values {startPoint, endPoint};
                std::vector<int> rows{i, endRow};
                LookupTable::interpolatePoints(values, rows, j);
                i = endRow - 1; //skip all the rows it just defined
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
void LookupTable::interpolatePoints(std::vector<double> values, std::vector<int>rows, int col) {
    double startVal = values[0];
    double endVal = values[1];
    int startRow = rows[0];
    int endRow = rows[1];
    if(mode == LINEAR) {
        if (endRow == startRow) {
            throw std::invalid_argument("Infinite increase in angle");
        }
        double m = (endVal - startVal) / (endRow - startRow);
        //Identify the points 
        double c = startVal - m * startRow;
        //Compute the next values in the column
        for (int i = startRow; i <endRow; i++) {
            csvData(i, col) = m * i + c;
        }
    }

}


double LookupTable::getPosition(int jointNo, double index) {
    /*
    Returns the joint position at the specified gait index
    Inputs:
        -jointNo: The joint number of the desired joint
            X2Params:
            0 -- LHip
            1 -- LKnee
            2 -- RHip
            3 -- RKnee
        -index: The Gait index of walk cycle
    Outputs:
        result: Double integer containing the singular value of
            the angle at the gait index.
    */
    int col = jointNo;

    if(index >= pointNo)
    {
        index = index - pointNo;
    } else if(index < 0) 
    {
        throw std::invalid_argument("index must be positive!");   
    }

    if(floor(index) == index)
    {
   
        //The index is a whole number, and greater than 0, interpolate
        int row;
        if(index > pointNo)
        {
            row = std::fmod(index, pointNo);
        } else 
        {
            row = floor(index);
        }
        //The data must be interpolated linearally
        return csvData(row, col);
    } else 
    {
        //Interpolate linearly
        int upIndex;
        int lowIndex;
        lowIndex = floor(index);
        upIndex = ceil(index);
        if(upIndex >= pointNo) 
        {
            //Identify the uupper
            upIndex = std::fmod(upIndex, pointNo);
        }
        double upVal = csvData(upIndex, col);        
        double lowVal = csvData(lowIndex, col);
        double m = (upVal - lowVal); //Change in one index
        double result = lowVal + m * (index - floor(index)); 
        return result;
    }
}
