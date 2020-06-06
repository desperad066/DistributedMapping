#!/bin/bash

# Params
isDebug=1;
solver=0; #GN
numSubgraphs=(2 4 9 16 25 36 49); # Num of subgraphs
datasets=('2robots' '4robots' '9robots' '16robots' '25robots' '36robots' '49robots') # Datasets on which we'll experiment
datasetDir='../../data/blocks_world/'

# make result directory
mkdir results_blocks_world

count=0;
# Run experiments on datasets
for dataset in ${datasets[@]}; 
do	
	#Run code
        echo "---------------------"
	resultDir='results_blocks_world/'$dataset
	admm_arguments='--dataDir '$datasetDir$dataset/' --nrRobots '${numSubgraphs[$count]}''
	../build/runDistributedMapper $admm_arguments 
	((count++))
        echo "---------------------"
done






