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
```

##  .g2o -> .txt(kitti format)
After compilation, execute 
```
./g2oToKitti -i [inputFilename(like 0.g2o)] -o [outputFilename(like 0.txt)]
evo_traj kitti [outputFilename(like 0.txt)] -p --plot_mode=xz
evo_traj kitti [outputFilename(like graph.txt)] --ref [outputFilename(like initial.txt)] -p --plot_mode=xz
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
