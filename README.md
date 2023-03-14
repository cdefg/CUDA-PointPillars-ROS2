# Modified by cdefg
THIS repository is not NVIDIA's original repository.

# Setup and Run

Prerequisites:
ROS2(tested on galactic, Ubuntu 22.04LTS)  
rclcpp  
sensor_msgs  
CUDA >= 11.3  
TensorRT >= 8.4

To compile:  
```
colcon build
```

# Essential Info to adapt to ROS2 system

## model IO
Original repo uses data from .bin binary files. But to build a stream processing package, this package adapt loadData() function from main.cpp.
The PointPillars model input takes serialized lidar data. the data is serialized point by point. The points are featured and the domains must be 'x', 'y', 'z', 'intensity'. As the PointPillars algorithm input only takes original $point$ infomation, the only thing we should do is to feed the PointCloud2 data (from ROS2) and size (row_step*height, stated from PointCloud2 msg in ros.org) into data C++ pointer. 

## ament building system for CUDA
Another tough thing to adapt to ROS2 is to build with CUDA, especially ROS2's ament system. 
Yet finally I succeeded in editing CMakeLists.txt and build with colcon build command.

### for cuda
```
find_package(CUDA REQUIRED)
```

### Get right path for tensorrt

```
set(TENSORRT_INCLUDE_DIRS /usr/include/x86_64-linux-gnu/)
set(TENSORRT_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)
```

### Link use `cuda_add_executable` or `cuda_add_library`

```
cuda_add_executable(pc_process src/pc_process.cpp src/cuda_pp_ros.cpp
src/cuda_pp_ros.cpp
src/pillarScatter.cpp
src/pointpillar.cpp
src/postprocess.cpp
src/preprocess.cpp

src/postprocess_kernels.cu
src/pillarScatterKernels.cu
src/preprocess_kernels.cu
)
```

# ORIGINAL Essentials Info from Nv's repo
# PointPillars Inference with TensorRT

Original Readme.md file can be found at
https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars  
$Below$ $is$ $lite$ $version.$

This repository contains sources and model for [pointpillars](https://arxiv.org/abs/1812.05784) inference using TensorRT.
The model is created with [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) and modified with onnx_graphsurgeon.

Overall inference has four phases:

- Convert points cloud into 4-channle voxels
- Extend 4-channel voxels to 10-channel voxel features
- Run TensorRT engine to get 3D-detection raw data
- Parse bounding box, class type and direction

## Model && Data

The demo use the velodyne data from KITTI Dataset.
The onnx file can be converted from [pre-trained model](https://drive.google.com/file/d/1wMxWTpU1qUoY3DsCH31WJmvJxcjFXKlm/view) with given script under "./tool".

### Prerequisites

To build the pointpillars inference, **TensorRT** with PillarScatter layer and **CUDA** are needed. PillarScatter layer plugin is already implemented as a plugin for TRT in the demo.

## Note

- GenerateVoxels has random output since GPU processes all points simultaneously while points selection for a voxel is random.
- The demo will cache the onnx file to improve performance. If a new onnx will be used, please remove the cache file in "./model".
- MAX_VOXELS in params.h is used to allocate cache during inference. Decrease the value to save memory.

## References

- [PointPillars: Fast Encoders for Object Detection from Point Clouds](https://arxiv.org/abs/1812.05784)
