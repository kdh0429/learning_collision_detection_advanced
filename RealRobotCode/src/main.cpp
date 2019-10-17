#include "UtilFunction.h"
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;

int main(int argc, char* argv[])
{
    float Input[155]= {0.0};
    float Output[2]= {0.0};
    CEstimateCollisionNN NN;

    fstream classFile("/home/kim/dusan_ws/Real_Robot_Code/2nd/build_ws_impact_cut/TestingDivide/Testing_data_136.csv");
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
