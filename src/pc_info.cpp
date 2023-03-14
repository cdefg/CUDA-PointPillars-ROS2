#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
// #include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

using sensor_msgs::msg::PointCloud2;

typedef sensor_msgs::msg::PointCloud2 PointCloud2;

class MsgExchangeClass : public rclcpp::Node
{
private:
  /* data */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  // rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr pub_;

  void callback(const PointCloud2::SharedPtr msg){
    int height = msg->height;
    int width  = msg->width;
    std::cout << "this point cloud has height " << height << " and width " << width << std::endl;
    int point_step = msg->point_step;
    int row_step = msg->row_step;
    std::cout << "row_step: " << row_step << "  point_step: " << point_step << std::endl;

    sensor_msgs::msg::PointCloud out_pc;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pc);

    for (int i = 0; i < out_pc.points.size(); i++){
      if (i < 5){
        std::cout << out_pc.points[i].x << "," << out_pc.points[i].y << "," << out_pc.points[i].z << "." << std::endl; 
      }
    }

    // point step = height = line num (32 line)
    // height * width = row_step??
    std::cout << "----------" << std::endl;
  }
public:
  MsgExchangeClass():Node("pc_process")
  {
    sub_ = this->create_subscription<PointCloud2>("rslidar_points", 10, std::bind(&MsgExchangeClass::callback, this, std::placeholders::_1));
    // pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>("objects", 10);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MsgExchangeClass>());
  rclcpp::shutdown();
  return 0;
}
