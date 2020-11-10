#include <ros/ros.h>
#include "YoloClassification.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_classification");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  yolo_classification::YoloClassification node(n, pn);
  
  ros::spin();
}