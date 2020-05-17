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
    CommandArgs arg;
    arg.param("i", inputFilename, "0.g2o", "input g2o filename");
    arg.param("o", outputFilename, "0_noise.g2o", "noised g2o filename");
//    arg.param("noise", noiseFilename, "-", "after noise filename");
//    arg.param("optim", optimFilename, "-", "after optim filename");
//    arg.param("kitti", kittiFilename, "-", "after kitti filename");
    arg.parseArgs(argc, argv);

    Kitti kitti(inputFilename);
    kitti.readG2OandNoise(inputFilename, outputFilename);
    return 0;
}
