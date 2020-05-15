#include "kitticlass.h"


Kitti::Kitti(string inFilename)
{
    fileInputStream_ = ifstream(inFilename.c_str());
    string line;
    int index = 0;
    while(getline(fileInputStream_, line))
        index++;
    num_ = index;
    fileInputStream_.clear();
    fileInputStream_.seekg(0);
}

void Kitti::generateV_E(int startIndex, int endIndex)
{
    vertices.clear();
    odometryEdges.clear();
    edges.clear();

    vector<double> pose;
    vector<Eigen::Isometry3d> poseT;
    string line;
    int index = 0;
    while(startIndex != 0 && getline(fileInputStream_, line)){
        if(++index == startIndex)
            break;
    }
    while(getline(fileInputStream_, line)) {
        stringstream ss;
        ss << line;
        Eigen::Isometry3d tempT = Eigen::Isometry3d::Identity();
        Eigen::Matrix3d tempRotateMatrix;
        Eigen::Vector3d tempTranslation;
        if(!ss.eof()) {
            double temp;
            while(ss >> temp)
                pose.push_back(temp);
        }
        tempRotateMatrix << pose[0], pose[1], pose[2], pose[4], pose[5], pose[6], pose[8], pose[9], pose[10];
        tempTranslation << pose[3], pose[7], pose[11];
        tempT.rotate(tempRotateMatrix);
        tempT.pretranslate(tempTranslation);
        poseT.push_back(tempT);
        pose.clear();
        if(++index == endIndex)
            break;
    }
    fileInputStream_.clear();
    fileInputStream_.seekg(0);

    std::vector<double> noiseTranslation;
    std::vector<double> noiseRotation;

    if (noiseTranslation.size() == 0) {
      cerr << "using default noise for the translation" << endl;
      noiseTranslation.push_back(0.2);
      noiseTranslation.push_back(0.2);
      noiseTranslation.push_back(0.2);
    }
    if (noiseRotation.size() == 0) {
      cerr << "using default noise for the rotation" << endl;
      noiseRotation.push_back(0.005);
      noiseRotation.push_back(0.005);
      noiseRotation.push_back(0.005);
    }

    Eigen::Matrix3d transNoise = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
      transNoise(i, i) = std::pow(noiseTranslation[i], 2);

    Eigen::Matrix3d rotNoise = Eigen::Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
      rotNoise(i, i) = std::pow(noiseRotation[i], 2);

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    information.block<3,3>(0,0) = transNoise.inverse();
    information.block<3,3>(3,3) = rotNoise.inverse();

    int id = 0;
    for (size_t f = 0; f < poseT.size(); ++f){
        VertexSE3* v = new VertexSE3;
        v->setId(id++);
        v->setEstimate(poseT[f]);
        vertices.push_back(v);
    }

    // generate odometry edges
    for (size_t i = 1; i < vertices.size(); ++i) {
      VertexSE3* prev = vertices[i-1];
      VertexSE3* cur  = vertices[i];
      Eigen::Isometry3d t = prev->estimate().inverse() * cur->estimate();
      EdgeSE3* e = new EdgeSE3;
      e->setVertex(0, prev);
      e->setVertex(1, cur);
      e->setMeasurement(t);
      e->setInformation(information);
      odometryEdges.push_back(e);
      edges.push_back(e);
    }

    // generate loop closure edges
    for (size_t f = 3; f < vertices.size(); ++f) {
        VertexSE3* from = vertices[f];
        for (size_t n = 1; n <= 3; ++n) {
          VertexSE3* to   = vertices[f-n];
          Eigen::Isometry3d t = from->estimate().inverse() * to->estimate();
          EdgeSE3* e = new EdgeSE3;
          e->setVertex(0, from);
          e->setVertex(1, to);
          e->setMeasurement(t);
          e->setInformation(information);
          edges.push_back(e);
        }
    }

    GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> transSampler;
    transSampler.setDistribution(transNoise);
    GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d> rotSampler;
    rotSampler.setDistribution(rotNoise);

    bool randomSeed = true;
    if (randomSeed) {
      std::random_device r;
      std::seed_seq seedSeq{r(), r(), r(), r(), r()};
      vector<int> seeds(2);
      seedSeq.generate(seeds.begin(), seeds.end());
      cerr << "using seeds:";
      for (size_t i = 0; i < seeds.size(); ++i)
        cerr << " " << seeds[i];
      cerr << endl;
      transSampler.seed(seeds[0]);
      rotSampler.seed(seeds[1]);
    }

    for (size_t i = 0; i < edges.size(); ++i) {
      EdgeSE3* e = edges[i];
      Eigen::Quaterniond gtQuat = (Eigen::Quaterniond)e->measurement().linear();
      Eigen::Vector3d gtTrans = e->measurement().translation();

      Eigen::Vector3d quatXYZ = rotSampler.generateSample();
      double qw = 1.0 - quatXYZ.norm();
      if (qw < 0) {
        qw = 0.;
        cerr << "x";
      }
      Eigen::Quaterniond rot(qw, quatXYZ.x(), quatXYZ.y(), quatXYZ.z());
      rot.normalize();
      Eigen::Vector3d trans = transSampler.generateSample();
      rot = gtQuat * rot;
      trans = gtTrans + trans;

      Eigen::Isometry3d noisyMeasurement = (Eigen::Isometry3d) rot;
      noisyMeasurement.translation() = trans;
      e->setMeasurement(noisyMeasurement);
    }

    for (size_t i =0; i < odometryEdges.size(); ++i) {
      EdgeSE3* e = edges[i];
      VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
      VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
      HyperGraph::VertexSet aux;
      aux.insert(from);
      e->initialEstimate(aux, to);
    }
    cerr << "Vetices Num: " << vertices.size() << endl;
    cerr << "Edge Num: " << edges.size() << endl;
}

void Kitti::write(string outFilename)
{
    ofstream fileOutputStream;
    if (outFilename != "-") {
      cerr << "Writing into " << outFilename << endl;
      fileOutputStream.open(outFilename.c_str());
    } else {
      cerr << "writing to stdout" << endl;
    }

    string vertexTag = Factory::instance()->tag(vertices[0]);
    string edgeTag = Factory::instance()->tag(edges[0]);

    ostream& fout = outFilename != "-" ? fileOutputStream : cout;
    for (size_t i = 0; i < vertices.size(); ++i) {
      VertexSE3* v = vertices[i];
      fout << vertexTag << " " << v->id() << " ";
      v->write(fout);
      fout << endl;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
      EdgeSE3* e = edges[i];
      VertexSE3* from = static_cast<VertexSE3*>(e->vertex(0));
      VertexSE3* to = static_cast<VertexSE3*>(e->vertex(1));
      fout << edgeTag << " " << from->id() << " " << to->id() << " ";
      e->write(fout);
      fout << endl;
    }
}

void Kitti::optimization(string inFilename, string outFileame)
{
    ifstream fin(inFilename);
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    //LinearSolver->BlockSolver->Algorithm
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {//Vertex
            // SE3 顶点
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;
            //vertex : Id; data
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {//Edge
            // SE3-SE3 边
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2;     // 关联的两个顶点
            fin >> idx1 >> idx2;
            //edge : Id; two vertex; data
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;
    optimizer.save(outFileame.c_str());
}

void Kitti::g2o_kitti(string inFilename, string outFilename)
{
    ifstream fin(inFilename);
    vector<Eigen::Isometry3d> poseT;
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {//Vertex
            // SE3 顶点
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            double data[7];
            double index = 0;
            fin >> index;
            //vertex : Id; data
            v->setId(index);
            for(int i = 0; i < 7; i++)
                fin >> data[i];
            Eigen::Quaterniond qt(data[6], data[3], data[4], data[5]);
            Eigen::Vector3d tr(data[0], data[1], data[2]);
            Eigen::Isometry3d tempT(qt);
            tempT.translation() = tr;
            poseT.push_back(tempT);
        } else if (name == "EDGE_SE3:QUAT") {
            continue;
        }
        if (!fin.good()) break;
    }
    ofstream fileOutputStream;
    if (outFilename != "-") {
      cerr << "Writing into " << outFilename << endl;
      fileOutputStream.open(outFilename.c_str());
    } else {
      cerr << "writing to stdout" << endl;
    }
    ostream& fout = outFilename != "-" ? fileOutputStream : cout;

    for (Eigen::Isometry3d T: poseT)
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 4; j ++)
                if(i==2 && j==3)
                    fout << T(i,j) << endl;
                else
                    fout << T(i,j) << " ";
}
