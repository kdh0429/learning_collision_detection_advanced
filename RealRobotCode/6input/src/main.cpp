#include "UtilFunction.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

int main(int argc, char* argv[])
{
    float Input[155]= {0.0};
    double Output[2]= {0.0};
    CEstimateCollisionNN NN;

    fstream classFile("/home/kim/dusan_ws/2nd/RealRobotCode/TestingDivideProcess/Testing_data_6.csv");
    string line;

    string data_string;
    
    

    int line_idx = 0;
    int idx = 0;
    while (getline(classFile, line,'\n')) // there is input overload classfile
    {
        istringstream iss(line);
        idx = 0;
        while (getline(iss, data_string,','))
        {
            if (idx < 155)
            {
            Input[idx] = stod(data_string);
            }
            if (idx == 158)
            {
                line_idx++;
                //if(line_idx == 1)
                    NN.EstimateCollision(Input, Output);
            }
            idx++;
        }

    }
	
    return 0;
}