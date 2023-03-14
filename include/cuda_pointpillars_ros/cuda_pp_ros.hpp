#ifndef CUDA_PP_ROS_H_
#define CUDA_PP_ROS_H_

#include <sensor_msgs/msg/point_cloud2.hpp>

void Getinfo(void);
void infer(sensor_msgs::msg::PointCloud2::SharedPtr msg);

#endif//CUDA_PP_ROS_H_