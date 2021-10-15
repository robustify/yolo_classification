#include "SyncedYoloData.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

namespace yolo_classification
{

  SyncedYoloData::SyncedYoloData(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(n, "image_rect", 5));
    sub_objects_.reset(new message_filters::Subscriber<YoloObjectArray>(n, "yolo_objects", 5));
    sync_yolo_data_.reset(new message_filters::Synchronizer<YoloSyncPolicy>(YoloSyncPolicy(10), *sub_img_, *sub_objects_));
    sync_yolo_data_->registerCallback(boost::bind(&SyncedYoloData::recvSyncedData, this, _1, _2));

    pub_output_image_ = n.advertise<sensor_msgs::Image>("yolo_image", 1);
  }

  void SyncedYoloData::recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const YoloObjectArrayConstPtr& object_msg)
  {
    Mat raw_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

    for (auto& bbox : object_msg->objects) {
      cv::Point2d corner(bbox.x, bbox.y - 5);
      rectangle(raw_img, cv::Rect(bbox.x, bbox.y, bbox.w, bbox.h), Scalar(0, 255, 0), 3);
      putText(raw_img, bbox.label, corner, FONT_HERSHEY_DUPLEX, 0.75, Scalar(255, 255, 255));
    }

    sensor_msgs::ImagePtr output_img_msg = cv_bridge::CvImage(img_msg->header, "bgr8", raw_img).toImageMsg();
    pub_output_image_.publish(output_img_msg);
  }

}