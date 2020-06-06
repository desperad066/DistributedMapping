#include <ADMM.h>

#define USE_L2_NORM 1

using namespace std;
using namespace gtsam;

/********************************************************************************************/
void
ADMM::logResult(int iter, double p_res, double d_res, double time_taken){

  // Create directories if they do not exist to save intermediate graphs
  logDir_ = outputDir_ + ("/intermediateGraphs/");
  if( !(boost::filesystem::exists(logDir_))){
    boost::filesystem::create_directory(logDir_);
  }

  subgraphLogDir_ = outputDir_ + ("/intermediateSubGraphs/");
  if( !(boost::filesystem::exists(subgraphLogDir_))){
    boost::filesystem::create_directory(subgraphLogDir_);
  }

  // Pack results into a single Value structure
  Values intermediateResult;
  for(size_t i=0; i<subinitials_.size(); i++){
    for (const Values::ConstKeyValuePair &keyValue: subinitials_[i]){
      if(!intermediateResult.exists(keyValue.key))
        intermediateResult.insert(keyValue.key, keyValue.value);
    }

    if(logCount_ > logCountThresh_){
    string subgraph_filepath = subgraphLogDir_ + boost::lexical_cast<string>(iter) + "_" + boost::lexical_cast<string>(i)  + ".g2o";
    //writeG2o(subgraphs_[i], subinitials_[i], subgraph_filepath);
    }
  }

  if(logCount_ > logCountThresh_){
  string filepath = logDir_ + boost::lexical_cast<string>(iter) + ".g2o";
  //writeG2o(fullGraph_, intermediateResult, filepath);
  logCount_ = 0;
  }

  //double admmError = 2*fullGraph_.error(intermediateResult);
  //*residualLog_ << iter << ", " << p_res << ", " << d_res << ", " << rho_ << ", " << admmError <<  "," << time_taken << endl;
  *residualLog_ << iter << ", " << p_res << ", " << d_res << ", " << rho_ << ", " <<  "," << time_taken << endl;

  if(verbosity_ >= DEBUG)
    //cout << "ADMM Error: " << admmError << endl;
    cout << "ADMM Error: " << 0 << endl;
}

/********************************************************************************************/
// Flagged Initialization
void
ADMM::FlaggedInitialization(){

//  GaussNewtonParams params;
//  params.setVerbosity("ERROR");
//  params.setRelativeErrorTol(-1e+10);
//  params.setAbsoluteErrorTol(-1e+10);
//  params.setMaxIterations(3);
//  int num_subgraphs = subgraphs_.size();
//  int nrSeparators = separators_.size()/3;

//  SharedNoiseModel rhoNoiseModel = noiseModel::Isotropic::Variance(3,2/rho_);

//  vector<Values> initial_old(subinitials_.size(), Values());
//  for(int sub_id =0; sub_id < num_subgraphs; sub_id++)
//    initial_old[sub_id] = Values(subinitials_[sub_id]);

//  vector<int> initFlag(subinitials_.size(), 0); // 0: subgraph is not optimized yet, 1: subgraph to be optimized in next iteration, 2: subgraph optimized
//  initFlag[0] = 1;

//  /// Keep optimizing un-initialized subgraphs
//  while(1){
//    bool optimizedSomething = false; // check if it has optimized atelast one sub-graph
//    for(int sub_id =0; sub_id < num_subgraphs; sub_id++){
//      if(initFlag[sub_id] == 1){
//        if (verbosity_ >= DEBUG)
//          printf("Flagged Initialization of %d\n", sub_id);
//        optimizedSomething = true;

//        // Create the local problem
//        NonlinearFactorGraph sub_graph = subgraphs_[sub_id];
//        Values sub_initial = initial_old[sub_id];

//        // add penalty factors for each constraint (i.e., for each separator)
//        for(int i=0; i< nrSeparators; i++){

//          if (separators_[3*i+1] == sub_id){ // behaves like f_x
//            int key = separators_[3*i]; // separator node symbol in the current subgraph

//            int graph_index = separators_[3*i+2]; // index of the neighboring subgraph

//            if(initFlag[graph_index] ==2){ // Add the BiasedPriorFactor if the neighboring graph is already initialized
//              Vector u_k = Vector3(uall_k[3*i], uall_k[3*i+1], uall_k[3*i+2]);
//              Pose3 pose_z = initial_old[graph_index].at<Pose3>(key);
//              sub_graph.add(BiasedPriorFactor<Pose3, Vector>(key, pose_z, -u_k, rhoNoiseModel));
//            }
//            else{
//              initFlag[graph_index] = 1; // Optimize it in the next iteration
//            }
//          }
//          else if (separators_[3*i + 2] == sub_id) // behaves like g_z
//          {
//            int key = separators_[3*i]; //separator node symbol in the current subgraph
//            int graph_index = separators_[3*i+1]; //index of the neighboring subgraph

//            if(initFlag[graph_index] ==2){ // Add the BiasedPriorFactor if the neighboring graph is already initialized

//              Vector u_k = Vector3(uall_k[3*i], uall_k[3*i+1], uall_k[3*i+2]);
//              Pose3 pose_x = initial_old[graph_index].at<Pose3>(key);
//              sub_graph.add(BiasedPriorFactor<Pose3, Vector>(key, pose_x, u_k, rhoNoiseModel));
//            }
//            else{
//              initFlag[graph_index] = 1; // Optimize it in the next iteration
//            }
//          }
//        }

//        if(solver_ == LAGO){
//          subinitials_[sub_id] = lago::initialize(sub_graph, false);
//        }
//        else if(solver_ == GN){
//          GaussNewtonOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
//          subinitials_[sub_id] = optimizer_subgraph.optimize();
//        }
//        else if(solver_ == LM){
//          cout << endl << "----------------" << endl;
//          LevenbergMarquardtParams params;
//          params.setlambdaInitial(1e-9);
//          params.setVerbosityLM("TRYLAMBDA");

//          LevenbergMarquardtOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
//          subinitials_[sub_id] = optimizer_subgraph.optimize();
//          cout << endl << "----------------" << endl;
//        }
//        else{
//          printf("ERROR: Specify solver type\n");
//        }

//        initFlag[sub_id] = 2; // subGraph initialized
//      }
//    }

//    if(!optimizedSomething){ // Break if we haven't optimized anything
//      if (verbosity_ >= DEBUG)
//        printf("Done initializing\n");
//      break;
//    }
//  } // end of loop over subgraphs

}


/********************************************************************************************/
// ADMM Iteration
pair<double, double>
ADMM::iterate(int iter){
  vector<VectorValues> subgraph_gradient(subgraphs_.size(), VectorValues());
  GaussNewtonParams params;
  if(iter == 0)
  {
    params.setRelativeErrorTol(-1e+10);
    params.setAbsoluteErrorTol(-1e+10);
    params.setMaxIterations(3);
  }

  size_t num_subgraphs = subgraphs_.size(); //Number of robots
  size_t nrSeparators = separators_.size()/2; //Separators

  SharedNoiseModel rhoNoiseModel = noiseModel::Isotropic::Variance(6,2/rho_);

  vector<Values> initial_old(subinitials_.size(), Values());
  for(size_t sub_id =0; sub_id < num_subgraphs; sub_id++)
    initial_old[sub_id] = Values(subinitials_[sub_id]);
  for(size_t sub_id =0; sub_id < num_subgraphs; sub_id++){
    if (!isParallel_)
      initial_old = subinitials_;

    NonlinearFactorGraph sub_graph = subgraphs_[sub_id];
    Values sub_initial = initial_old[sub_id];

    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12);
    for(size_t i=0; i < nrSeparators; i++){
      if (separators_[2*i].first == sub_id){
          gtsam::Symbol fromkey = separators_[2*i].second;
          size_t fromGraph_index = separators_[2*i].first;
          gtsam::Symbol tokey = separators_[2*i+1].second;
          size_t toGraph_index = separators_[2*i+1].first;
          Pose3 fromPose = initial_old[fromGraph_index].at<Pose3>(fromkey);
          Pose3 toPose = initial_old[toGraph_index].at<Pose3>(tokey);

          Eigen::Matrix<double, 6, 1> u_k;
          u_k << uall_k[6*i], uall_k[6*i+1], uall_k[6*i+2], uall_k[6*i+3], uall_k[6*i+4], uall_k[6*i+5];
          Pose3 between = measures_[i];
          sub_graph.add(BiasedBetweenFactor<Pose3, Vector>(fromkey, tokey, between, u_k, rhoNoiseModel));

          if(sub_id != 0)
              sub_graph.add(PriorFactor<Pose3>(tokey, toPose ,priorModel));
          if(!sub_initial.exists(tokey))
              sub_initial.insert(tokey, toPose);
      }
      else if (separators_[2*i + 1].first == sub_id)
      {
          cout << "Seperators in: " << i << "to" << endl;
          gtsam::Symbol fromkey = separators_[2*i].second;
          size_t fromGraph_index = separators_[2*i].first;
          gtsam::Symbol tokey = separators_[2*i+1].second;
          size_t toGraph_index = separators_[2*i+1].first;
          Pose3 fromPose = initial_old[fromGraph_index].at<Pose3>(fromkey);
          Pose3 toPose = initial_old[toGraph_index].at<Pose3>(tokey);

          Eigen::Matrix<double, 6, 1> u_k;
          u_k << uall_k[6*i], uall_k[6*i+1], uall_k[6*i+2], uall_k[6*i+3], uall_k[6*i+4], uall_k[6*i+5];
          Pose3 between = measures_[i];
          sub_graph.add(BiasedBetweenFactor<Pose3, Vector>(fromkey, tokey, between, u_k, rhoNoiseModel));

          if(sub_id != 0)
              sub_graph.add(PriorFactor<Pose3>(fromkey, fromPose ,priorModel));
          if(!sub_initial.exists(fromkey))
              sub_initial.insert(fromkey, fromPose);
      }
    }

    if(solver_ == LAGO){
      subinitials_[sub_id] = lago::initialize(sub_graph, false);
    }
    else if(solver_ == GN){
      GaussNewtonOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
      subinitials_[sub_id] = optimizer_subgraph.optimize();
    }
    else if(solver_ == LM){
      LevenbergMarquardtParams params;
      params.setlambdaInitial(1e-7);
      LevenbergMarquardtOptimizer optimizer_subgraph(sub_graph, sub_initial, params);
      subinitials_[sub_id] = optimizer_subgraph.optimize();
    }
    else{
      printf("ERROR: Specify solver type\n");
    }
    std::cout << "Optimization done" << std::endl;
    subgraph_gradient[sub_id] = subgraphs_[sub_id].linearize(subinitials_[sub_id])->gradientAtZero();
  }

  double p_res = 0, d_res = 0;

  NonlinearFactorGraph constraintsGraph;
  for(size_t i=0; i<nrSeparators; i++){
    gtsam::Symbol key1 = separators_[2*i].second;
    gtsam::Symbol key2 = separators_[2*i+1].second;
    size_t sub_map1 = separators_[2*i].first;
    size_t sub_map2 = separators_[2*i+1].first;
    Pose3 pose_x = subinitials_[sub_map1].at<Pose3>(key1);
    Pose3 pose_z = subinitials_[sub_map2].at<Pose3>(key2);

    Matrix J_x, J_z;
    Pose3 between_x_z = pose_x.between(pose_z);
    Pose3 between_old = measures_[i];
    Pose3 difference = between_old.between(between_x_z, J_x, J_z);
    Vector b = Pose3::Logmap(difference);
    std::cout << "b:" << b.transpose() << std::endl;
    Eigen::Matrix<double, 6, 1> u_k;
    u_k << uall_k[6*i], uall_k[6*i+1], uall_k[6*i+2], uall_k[6*i+3], uall_k[6*i+4], uall_k[6*i+5];
    Vector y_k = (rho_/2) * u_k;
    Vector scaled_Jx = y_k.transpose()*J_x;
    Vector scaled_Jz = y_k.transpose()*J_z;
    std::cout << "scaled_Jx:" << scaled_Jx.transpose() << std::endl;
    std::cout << "scaled_Jz:" << scaled_Jz.transpose() << std::endl;
    if(subgraph_gradient[sub_map1].exists(key1)){
      subgraph_gradient[sub_map1].at(key1) += scaled_Jx;
    }
    else{
      subgraph_gradient[sub_map1].insert(key1, scaled_Jx);
    }

    if(subgraph_gradient[sub_map2].exists(key2)){
      subgraph_gradient[sub_map2].at(key2) += scaled_Jz;
    }
    else{
      subgraph_gradient[sub_map2].insert(key2, scaled_Jz);
    }


    p_res = p_res + b.norm();

    uall_k[6*i]   = uall_k[6*i]   + b[0];
    uall_k[6*i+1] = uall_k[6*i+1] + b[1];
    uall_k[6*i+2] = uall_k[6*i+2] + b[2];
    uall_k[6*i+3] = uall_k[6*i+3] + b[3];
    uall_k[6*i+4] = uall_k[6*i+4] + b[4];
    uall_k[6*i+5] = uall_k[6*i+5] + b[5];
  }
  for(size_t sub_id =0; sub_id < num_subgraphs; sub_id++){
    double gradient_norm = subgraph_gradient[sub_id].norm();
    d_res = d_res + gradient_norm * gradient_norm;
  }
  d_res = sqrt(d_res);

  return make_pair(p_res, d_res);
}

/********************************************************************************************/
// ADMM - return sub_initials (optimized), and a vector of dual variables
pair<vector<Values>, vector<double> >
ADMM::optimize(){
  double p_res, d_res;
  string residualLog = outputDir_ + "/residualLog.txt";
  residualLog_->open(residualLog.c_str(), fstream::out);

  if(verbosity_ >= DEBUG){
    cout << "========================================" << endl;
    cout << "Parameters:  "
            "\n-rho: " << rho_ << "(initial)" <<
            "\n-mu: "  << mu_ <<
            "\n-tau: "  << tau_ <<
            "\n-maxIter: "  << maxIter_ <<
            "\n-isParallel: "  << isParallel_ <<
            "\n-adaptivePenalty: "  << adaptivePenalty_ <<
            "\n-min_p_res_: "  << min_p_res_ <<
            "\n-min_d_res_: "  << min_d_res_ <<
            "\n-num_subgraphs: " << subgraphs_.size() <<
            "\n-solver-type: " << solver_ << endl;
  }

  logCount_ = 0, logCountThresh_ = 10;

  // Start ADMM Timer
  ADMM::Time admm_start = boost::posix_time::microsec_clock::local_time ();

  // Flagged initialization
  if (useFlaggedInitialization_)
    FlaggedInitialization();


  // run ADMM iterations
  for(int iter = 0; iter < maxIter_; iter++){

    // Start iter timer
    tic();

    if(verbosity_ >= DEBUG)
      cout << "============== Iter: " << iter << " ==============" << endl;

    // run single ADMM iteration
    boost::tie(p_res, d_res)  = iterate(iter);

    // Stop timer
    double time_duration = toc();

    if(adaptivePenalty_ && p_res > mu_ * d_res)
      rho_ = rho_ * tau_;
    if(adaptivePenalty_ && d_res > mu_ * p_res)
      rho_ = rho_ / tau_;

    if (verbosity_ >= LOG){
      logResult(iter, p_res, d_res, time_duration);
    }
    logCount_++;
    // print debug info
    if(verbosity_ >= DEBUG){
      printf("p_res(iter) : %g (%d) < %g: min_p_res_ \n",p_res,iter,min_p_res_);
      printf("d_res(iter) : %g (%d) < %g: min_d_res_ \n",d_res,iter,min_d_res_);
      cout << "rho: " << rho_ << " (tau=" << tau_ << ", mu=" << mu_ << ")" << endl;
      if(iter == maxIter_-1)
        cout << "Reached maxIter: " << maxIter_-1 << endl;
    }

    //check stopping conditions
    if(p_res < min_p_res_ && d_res < min_d_res_){

      if(verbosity_ >= DEBUG){
        cout << "========================================" << endl;
        printf("p_res(iter) : %g (%d) < %g: min_p_res_ \n",p_res,iter,min_p_res_);
        printf("d_res(iter) : %g (%d) < %g: min_d_res_ \n",d_res,iter,min_d_res_);
      }
      break;
    }
  }

  if(verbosity_ >= DEBUG){
    ADMM::Time admm_end = boost::posix_time::microsec_clock::local_time ();
    boost::posix_time::time_duration time_diff = admm_end - admm_start;
    cout <<  "Total time: " <<  time_diff.total_microseconds() << endl;
  }

  residualLog_->close();

  // unscale dual variables
  vector<double> y_k(uall_k.size(), 0.0f);
  for(size_t i = 0; i < y_k.size(); i++)
    y_k[i] =uall_k[i]*rho_; // unscaled dual variables

  // return ADMM primal and dual solution
  string robotNames = string("abcdefghijklmnopqrstuvwxyz");
  for(size_t index = 0; index < subinitials_.size(); index ++){
      Values sub_ini = subinitials_[index];
      KeyVector keys = sub_ini.keys();
      for(size_t keyIndex = 0; keyIndex < keys.size(); keyIndex++){
          Key key = keys.at(keyIndex);
          char robot = symbolChr(key);
          if(robot != robotNames[index]){
              subinitials_[index].erase(key);
          }
      }
  }

  return make_pair(subinitials_, y_k);
}
