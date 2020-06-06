// ADMM
#include <ADMM.h>
#include <ADMMUtils.h>

// GTSAM
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>

// Boost
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <fstream>
#include <sstream>

using namespace std;
using namespace gtsam;

// To be used when you already have the partitioning (bunch of g2o files + separator file)

/**
 * @brief readFullGraph reads the full graph if it is present in the directory, otherwise creates it
 * @param nrRobots is the number of robots
 * @param graphAndValuesVec contains the graphs and initials of each robot
 */
GraphAndValues readFullGraph(size_t nrRobots, // number of robots
                             vector<GraphAndValues> graphAndValuesVec  // vector of all graphs and initials for each robot
                             ){
  std::cout << "Creating fullGraph by combining subgraphs." << std::endl;

  // Combined graph and Values
  NonlinearFactorGraph::shared_ptr combinedGraph(new NonlinearFactorGraph);
  Values::shared_ptr combinedValues(new Values);

  // Iterate over each robot
  for(size_t robot = 0; robot < nrRobots; robot++){

      // Load graph and initial
      NonlinearFactorGraph graph = *(graphAndValuesVec[robot].first);
      Values initial = *(graphAndValuesVec[robot].second);

      // Iterate over initial and push it to the combinedValues, each initial value is present only once
      for(const Values::ConstKeyValuePair& key_value: initial){
          Key key = key_value.key;
          if(!combinedValues->exists(key))
            combinedValues->insert(key, initial.at<Pose3>(key));
        }

      // Iterate over the graph and push the factor if it is not already present in combinedGraph
      for(size_t ksub=0; ksub<graph.size(); ksub++){ //for each factor in the new subgraph
          bool isPresent = false;
          for(size_t k=0; k<combinedGraph->size(); k++){

              boost::shared_ptr<BetweenFactor<Pose3> > factorSub =
                  boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(graph.at(ksub));
              Key factorSubKey1 = factorSub->key1();
              Key factorSubKey2 = factorSub->key2();

              boost::shared_ptr<BetweenFactor<Pose3> > factorCombined =
                  boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(combinedGraph->at(k));
              Key factorCombinedKey1 = factorCombined->key1();
              Key factorCombinedKey2 = factorCombined->key2();

              // values don't match exactly that's why check with keys as well
              if(factorCombined->equals(*factorSub) || ((factorSubKey1 == factorCombinedKey1) && (factorSubKey2 == factorCombinedKey2))){
                  isPresent = true;
                  break;
                }
            }
          if(isPresent == false) // we insert the factor
            combinedGraph->add(graph.at(ksub));
        }
    }

  // Return graph and values
  return make_pair(combinedGraph, combinedValues);
}


int main(int argc, char* argv[])
{

  int num_subgraphs;
  string output_dir; // output_dir
  string input_dir; // input dir

  double rho = 0.1; // penalty parameter in augmented lagrangian
  double mu = 10.0; // parameter used to decide whether to increase or not the rho (adaptive penalty)
  double tau = 2.0; // if we have to increase rho, we do rho <- rho * tau, otherwise rho <- rho / tau
  int maxIter = 1000; // max number of ADMM iterations
  bool isParallel = false; // if true ADMM runs in parallel mode, this is usually less stable
  bool adaptivePenalty = true;
  bool useFlaggedInitialization = false;
  bool computeSubgraphGradient = true;
  int solver = 0; // GN
  double min_p_res = 0.1f;
  double min_d_res = 0.1f;

  // Verbosity
  int verbosity = 2;

  try{
    // Parse program options
    namespace po = boost::program_options;
    po::options_description desc("Options");
    desc.add_options()
        ("help", "Print help messages")
        ("input_dir,i",po::value<string>(&input_dir)->required(), "Input director (required)")
        ("num_subgraphs,n", po::value<int>(&num_subgraphs)->required(), "Num Subgraphs (required)")
        ("result_dir,r", po::value<string>(&output_dir)->required(), "Results Directory (required)")
        ("rho_val", po::value<double>(&rho), "rho value (default: 0.1)")
        ("mu_val,mu", po::value<double>(&mu), "mu value (default: 10.0)")
        ("tau_val,tau", po::value<double>(&tau), "tau value (default: 2.0)")
        ("max_iter", po::value<int>(&maxIter), "Maximum number of iterations (default: 1000)")
        ("adaptive_penalty,adpt_pen", po::value<bool>(&adaptivePenalty), "To use adaptive penalty or not (default: true)")
        ("use_flagged_init", po::value<bool>(&useFlaggedInitialization), "To use Flagged Initialization or not (default: true)")
        ("is_parallel", po::value<bool>(&isParallel), "To use parallel multiblock ADMM or not (default: false)")
        ("compute_subgraph_gradient", po::value<bool>(&computeSubgraphGradient), "To compute subgraph gradient  or not (default: true)")
        ("min_p_res", po::value<double>(&min_p_res), "min_p_res value (default: 0.01f)")
        ("min_d_res", po::value<double>(&min_d_res), "min_d_res value (default: 0.01f)")
        ("solver", po::value<int>(&solver), "Specify solver type (default: GN)")
        ("verbosity", po::value<int>(&verbosity), "Specify verbosity level (default: SILENT)");



    po::variables_map vm;
    try{
      po::store(po::parse_command_line(argc, argv, desc), vm); // can throw
      if ( vm.count("help")  ){ // --help option
        cout << "Partition and Run ADMM" << endl << desc << endl;
        return 0;
      }
      po::notify(vm); // throws on error, so do after help in case
    }
    catch(po::error& e){
      cerr << "ERROR: " << e.what() << endl << endl;
      cerr << desc << endl;
      return 1;
    }
  }
  catch(exception& e){
    cerr << "Unhandled Exception reached the top of main: "
              << e.what() << ", application will now exit" << endl;
    return 2;
  }

  // Create the output directory if it does not exist
  if( !(boost::filesystem::exists(output_dir))){
    boost::filesystem::create_directory(output_dir);
  }

  // Subgraph and Subinitials
  vector<NonlinearFactorGraph> sub_graphs;
  vector<Values> sub_initials;
  PriorFactor<Pose3> posePrior;

  // Read subgraphs
  std::vector<std::pair<int, gtsam::Symbol> > separators;
  std::vector<gtsam::Pose3> measures_;
  string robotName = string("abcdefghijklmnopqrstyvwxyz");
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise

  vector<GraphAndValues> graphAndValuesVec; // vector of all graphs and initials
  for(int sub_id = 0; sub_id < num_subgraphs; sub_id++)
  {
      std::cout << sub_id << std::endl;
    string filepath = input_dir + boost::lexical_cast<string>(sub_id) + string(".g2o");
    bool is3D = true;
    GraphAndValues readGraph = readG2o(filepath, is3D);
    std::cout << filepath <<"   read Done" << std::endl;
    NonlinearFactorGraph graph = *(readGraph.first);
    Values initial = *(readGraph.second);

    GraphAndValues graphAndValues =  make_pair(readGraph.first, boost::make_shared<Values>(initial));
    graphAndValuesVec.push_back(graphAndValues);
    PrintKeyVector(initial.keys());
    createSubgraphInnerAndSepEdges(robotName[sub_id], sub_id, graph, separators, measures_, false, true);
    size_t factorSize = graph.nrFactors();
    for(size_t index = 0; index < factorSize; index++){
        Key fromKey = graph[index]->keys().at(0);
        Key toKey = graph[index]->keys().at(1);
        char robot0 = symbolChr(fromKey);
        char robot1 = symbolChr(toKey);
        if(robot0 != robot1){
            graph[index]->print("\nFactor\n");
            graph.remove(index);
        }
    }
    if (sub_id == 0){
      Key firstKey = KeyVector(initial.keys()).at(0);
      posePrior = PriorFactor<Pose3>(firstKey, initial.at<Pose3>(firstKey),priorModel);
      graph.add(posePrior);
    }
    sub_graphs.push_back(graph);
    sub_initials.push_back(initial);
  }
  GraphAndValues fullGraphAndValues = readFullGraph(num_subgraphs, graphAndValuesVec);
  NonlinearFactorGraph fullGraph = *(fullGraphAndValues.first);

  // Read separators
//  string separator_file = input_dir + "separators.txt";
//  vector < vector <string> > separators_str;
//  ifstream infile;
//  infile.open(separator_file.c_str(), ifstream::in);

//  while (infile){
//    string s;
//    if (!getline( infile, s )) break;

//    istringstream ss(s);
//    vector <string> record;

//    while (ss){
//      string s;
//      if (!getline( ss, s, ',' )) break;
//      record.push_back(s);
//    }

//    separators_str.push_back( record );
//  }
//  if (!infile.eof()){
//    cerr << "File still not read completely!\n";
//  }

//  // Load the separators
//  vector <int>  separators;
//  for(size_t i=0; i < separators_str.size(); i++){
//    for(size_t j=0; j< separators_str[i].size(); j++){
//      separators.push_back(boost::lexical_cast<int>(separators_str[i][j]));
//    }
//  }


  // Read centralized graph from g2o file
//  string filepath = input_dir + "/fullGraph.g2o";
//  GraphAndValues readGraph = readG2o(filepath);
//  NonlinearFactorGraph graph = *(readGraph.first);
//  Values initial = *(readGraph.second);


  // Write down the G2o files
  for(size_t sub_id =0; sub_id < sub_initials.size(); sub_id++){
    string filepath = output_dir + "/" + boost::lexical_cast<string>(sub_id) + ".g2o";
    writeG2o(sub_graphs[sub_id], sub_initials[sub_id], filepath);
  }

  string filepath = output_dir + "/separators.txt";
  fstream separatorFile(filepath.c_str(), fstream::out);
  for(size_t i = 0; i < separators.size(); i += 2){
    separatorFile << separators[i].first <<  "," << separators[i].second <<  "," << separators[i+1].first  << "," << separators[i+1].second << endl;
  }
  separatorFile.close();

  // Run ADMM
  // Construct ADMM
  ADMM admm(rho,
            mu,
            tau,
            maxIter,
            isParallel,
            adaptivePenalty,
            useFlaggedInitialization,
            computeSubgraphGradient,
            min_p_res,
            min_d_res,
            output_dir);

  admm.setSolver(ADMM::Solver(solver));
  admm.setVerbosity(ADMM::Verbosity(verbosity));

  // Load structures
  admm.load(sub_graphs, sub_initials, separators, measures_);

  cout << "Running on " << input_dir << endl;
  // Run ADMM
  clock_t start = clock();
  pair<vector<Values>, vector<double> > primalDualResult = admm.optimize();
  clock_t end = clock();
  double times = (end-start);
  cout << "Total Time for all optimization----------: " << times/CLOCKS_PER_SEC*1000 << "ms" << endl;
  vector<Values> sub_results = primalDualResult.first;
  Values distributed;
  for(size_t i = 0; i< sub_results.size(); i++){
      for(const Values::ConstKeyValuePair& key_value: sub_results[i]){
          Key key = key_value.key;
          if(!distributed.exists(key))
            distributed.insert(key, sub_results[i].at<Pose3>(key));
        }
    }
  std::cout << "Distributed Error: " << fullGraph.error(distributed) << std::endl;



  // Pack ADMM results into a single Value structure, to check centralized error
  // logResults(sub_graphs, sub_results, graph, initial, posePrior, output_dir);
  logResults(sub_graphs, sub_results, posePrior, output_dir);
  return 0;
}
