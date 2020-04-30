// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
    string noiseFilename = "noise";
    string optimFilename = "result";
    string kittiFilename;
    int robotNum;
    CommandArgs arg;
    arg.param("i", inputFilename, "00.txt", "input true ground filename");
//    arg.param("noise", noiseFilename, "-", "after noise filename");
//    arg.param("optim", optimFilename, "-", "after optim filename");
//    arg.param("kitti", kittiFilename, "-", "after kitti filename");
    arg.param("num", robotNum, 2, "the number of robots");
    arg.parseArgs(argc, argv);

    Kitti kitti(inputFilename);
    kitti.generateV_E(0, kitti.indexNum());
    kitti.write("noise_whole.g2o");
    kitti.optimization("noise_whole.g2o", "result_whole.g2o");
    kitti.g2o_kitti("result_whole.g2o", "kitti_whole.txt");

    string cmd = "evo_traj kitti result_whole.txt " + inputFilename;
    for(int i = 0; i < robotNum; i++){
        kitti.generateV_E(kitti.indexNum()*i/robotNum, kitti.indexNum()*(i+1)/robotNum);
        noiseFilename = "noise_part_" + to_string(i) + ".g2o";
        kitti.write(noiseFilename);
        optimFilename = "result_part_" + to_string(i) + ".g2o";
        kitti.optimization(noiseFilename, optimFilename);
        kittiFilename = "kitti_part_" + to_string(i) + ".txt";
        kitti.g2o_kitti(optimFilename, kittiFilename);
        cmd += " "+kittiFilename;
    }
    cmd += " -p --plot_mode xz";
    system("export PATH=/home/desperado/anaconda3/bin/:$PATH");
    system(cmd.c_str());
    return 0;
}
