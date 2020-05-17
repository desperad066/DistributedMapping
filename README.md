# DistributedMapping

## Requirements
* [Eigen](http://eigen.tuxfamily.org)

* [g2o](https://github.com/RainerKuemmerle/g2o)

* [Sophus](https://github.com/strasdat/Sophus)

* [evo](https://github.com/MichaelGrupp/evo)(optional)

  you can install `evo` by `pip` simply, like
```
pip install evo --upgrade --no-binary evo
```


## Compilation
```
mkdir build
cd build
cmake ..
make -j4
./kitti(or ./sphere)
sudo make install 
```

##  .g2o -> .txt(kitti format)
After compilation, execute 
```
g2oToKitti -n [RobotNumber] -is [inputFilenameSuffix(like _optimized)] -os [outputFilenameSuffix(like _optimized)]
evo_traj kitti [outputFilename(like 0.txt)] -p --plot_mode=xz
```

For example, if there are files named `0.g2o`, `1.g2o`,`0_optimized.g2o`,`1_optimized.g2o` , you can execute

```
g2oToKitti -n 2 
g2oToKitti -n 2 -is _optimized -os _optimized
```
Then you will get `0.txt`,`1.txt`,`0_optimized.txt`,`1_optimized.txt`and execute
```
evo_traj kitti [yourFiles] -p --plot_mode=xz
```

## Data/1/

```
x.g2o :initial poses
x_optimized.g2o :optimized poses
x_optimizedTUM.txt :optimized posed(TUM format)
initial.g2o	:total initial poses
fullGraph_optimized :optimized and merged poses
centralizedxxx.g2o :merged and optimized poses
```
## Data/first_crash
You can use `distributed_mapper` to optimize them and get optimized result, then use `g2oToKitti` for visualization.