#ifndef CUDA_PP_ROS_H_
#define CUDA_PP_ROS_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

void Getinfo(void);
visualization_msgs::msg::MarkerArray infer(sensor_msgs::msg::PointCloud2::SharedPtr msg);

#endif//CUDA_PP_ROS_H_