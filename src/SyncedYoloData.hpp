#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <yolo_classification/YoloObjectArray.h>

// Filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

namespace yolo_classification
{

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, YoloObjectArray> YoloSyncPolicy;

  class SyncedYoloData
  {
    public:
      SyncedYoloData(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:

      void recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const YoloObjectArrayConstPtr& object_msg);

      boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_img_;
      boost::shared_ptr<message_filters::Subscriber<YoloObjectArray> > sub_objects_;
      boost::shared_ptr<message_filters::Synchronizer<YoloSyncPolicy> > sync_yolo_data_;
      ros::Publisher pub_output_image_;
  };
}