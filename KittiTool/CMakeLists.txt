project(use_kitti)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Eigen
include_directories("/usr/include/eigen3")

# sophus
#find_package(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})

# g2o
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

aux_source_directory(./src SRC_FILES)
include_directories(./include)

add_executable(kitti
    src/kitti.cpp
    src/kitticlass.cpp)
set_target_properties(kitti PROPERTIES OUTPUT_NAME kitti${EXE_POSTFIX})
target_link_libraries(kitti ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})

add_executable(sphere src/sphere.cpp)
set_target_properties(sphere PROPERTIES OUTPUT_NAME sphere${EXE_POSTFIX})
target_link_libraries(sphere ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})

add_executable(g2oToKitti
    src/g2otoKitti.cpp
    src/kitticlass.cpp)
set_target_properties(g2oToKitti PROPERTIES OUTPUT_NAME g2oToKitti${EXE_POSTFIX})
target_link_libraries(g2oToKitti ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})

add_executable(readG2OandNoise
    src/readG2OandNoise.cpp
    src/kitticlass.cpp)
set_target_properties(readG2OandNoise PROPERTIES OUTPUT_NAME readG2OandNoise${EXE_POSTFIX})
target_link_libraries(readG2OandNoise ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES})

install(TARGETS g2oToKitti RUNTIME DESTINATION bin)
