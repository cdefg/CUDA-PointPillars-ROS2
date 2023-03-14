#include "cuda_pointpillars_ros/cuda_pp_ros.hpp"
#include "cuda_runtime.h"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdio>

#include "params.h"
#include "pointpillar.h"

std::string Model_File = "/home/e404/perception_ws/src/cuda_pointpillars_ros/model/pointpillar.onnx";

void Getinfo(void)
{
  cudaDeviceProp prop;

  int count = 0;
  cudaGetDeviceCount(&count);
  printf("\nGPU has cuda devices: %d\n", count);
  for (int i = 0; i < count; ++i) {
    cudaGetDeviceProperties(&prop, i);
    printf("----device id: %d info----\n", i);
    printf("  GPU : %s \n", prop.name);
    printf("  Capbility: %d.%d\n", prop.major, prop.minor);
    printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
    printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
    printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
    printf("  warp size: %d\n", prop.warpSize);
    printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
    printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
    printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
  }
  printf("\n");
}

void infer(sensor_msgs::msg::PointCloud2::SharedPtr msg){
  cudaEvent_t start, stop;
  float elapsedTime = 0.0f;
  cudaStream_t stream = NULL;

  checkCudaErrors(cudaEventCreate(&start));
  checkCudaErrors(cudaEventCreate(&stop));
  checkCudaErrors(cudaStreamCreate(&stream));

  Params params_;
  std::vector<Bndbox> nms_pred;
  nms_pred.reserve(100);

  PointPillar pointpillar(Model_File, stream);

  float* points = (float*)msg->data.data();
  size_t height = msg->height;
  size_t width = msg->width;
  size_t row_step = msg->row_step;
  size_t length = row_step * height;
  size_t points_size = length/sizeof(float)/4;

  float *points_data = nullptr;
  unsigned int points_data_size = points_size * 4 * sizeof(float);
  
  checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
  checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
  checkCudaErrors(cudaDeviceSynchronize());

  cudaEventRecord(start, stream);

  pointpillar.doinfer(points_data, points_size, nms_pred);
  cudaEventRecord(stop, stream);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  std::cout<<"TIME: pointpillar: "<< elapsedTime <<" ms." <<std::endl;

  checkCudaErrors(cudaFree(points_data));

  std::cout<<"Bndbox objs: "<< nms_pred.size()<<std::endl;

  nms_pred.clear();

  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));
}