#ifndef _KITTI_CLASS_H_
#define _KITTI_CLASS_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/factory.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;
using namespace g2o;

class Kitti{
public:

    Kitti(string inFilename);
    int indexNum(){return num_;}
    void generateV_E(int startIndex, int endIndex);
    void write(string outFilename);
    void optimization(string inFilename, string outFileame);
    void g2o_kitti(string inFilename, string outFileame);
    void readG2OandNoise(string inFilename, string outFilename);
private:
    ifstream fileInputStream_;
    int num_;
    vector<VertexSE3*> vertices;
    vector<EdgeSE3*> odometryEdges;
    vector<EdgeSE3*> edges;

};
#endif
