#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <cstring>

#include <Eigen/Core>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/factory.h"

#include "kitticlass.h"
using namespace std;
using namespace g2o;

int main (int argc, char** argv)
{
    string inputFilename;
    string outputFilename;
    int robotNum;
    string inputSuffix;
    string outputSuffix;
    CommandArgs arg;
    //arg.param("i", inputFilename, "0.g2o", "input g2o filename");
    //arg.param("o", outputFilename, "0.txt", "output kitti filename");
    arg.param("n", robotNum, 2, "number of robots");
    arg.param("is", inputSuffix, "-", "input suffix");
    arg.param("os", outputSuffix, "-", "output suffix");
//    arg.param("noise", noiseFilename, "-", "after noise filename");
//    arg.param("optim", optimFilename, "-", "after optim filename");
//    arg.param("kitti", kittiFilename, "-", "after kitti filename");
    arg.parseArgs(argc, argv);
    Kitti kitti(inputFilename);
    for(int i = 0; i < robotNum; i++){
        string inName, outName;
        if(inputSuffix == "-")
            inName = to_string(i) + ".g2o";
        else
            inName = to_string(i) + inputSuffix + ".g2o";
        if(outputSuffix == "-")
            outName = to_string(i) + ".txt";
        else
            outName = to_string(i) + outputSuffix + ".txt";
        kitti.g2o_kitti(inName, outName);
    }
    return 0;
}
