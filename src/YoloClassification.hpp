#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <yolo_classification/YoloObjectArray.h>

#include <dynamic_reconfigure/server.h>
#include <yolo_classification/DarknetConfig.h>

#include <darknet_ros/darknet.h>

namespace yolo_classification
{

  class YoloClassification
  {
    public:
      YoloClassification(ros::NodeHandle n, ros::NodeHandle pn);

    private:
      void reconfig(DarknetConfig& config, uint32_t level);
      void recvImage(const sensor_msgs::ImageConstPtr& msg);
      void runDarknet(const cv::Mat& raw_img, std::vector<YoloObject>& darknet_bboxes);

      ros::Subscriber sub_image_;
      ros::Publisher pub_detections_;

      dynamic_reconfigure::Server<DarknetConfig> srv_;
      DarknetConfig cfg_;

      std::string darknet_cfg_file_;
      std::string darknet_weights_file_;
      network* net_;
      int skip_;

      static inline void rgbgr_image(image& im) {
        int i;
        for (i = 0; i < im.w * im.h; ++i) {
          float swap = im.data[i];
          im.data[i] = im.data[i+im.w*im.h*2];
          im.data[i+im.w*im.h*2] = swap;
        }
      }

      static inline image ipl_to_image(IplImage* src) {
        int h = src->height;
        int w = src->width;
        int c = src->nChannels;
        image im = make_image(w, h, c);
        unsigned char *data = (unsigned char *)src->imageData;
        int step = src->widthStep;
        int i, j, k;

        for (i = 0; i < h; ++i) {
            for (k= 0; k < c; ++k) {
                for (j = 0; j < w; ++j) {
                    im.data[k * w * h + i * w + j] = data[i * step + j * c + k] / 255.0;
                }
            }
        }
        return im;
      }

      static inline IplImage *image_to_ipl(image im) {
        int x,y,c;
        IplImage *disp = cvCreateImage(cvSize(im.w, im.h), IPL_DEPTH_8U, im.c);
        int step = disp->widthStep;
        for (y = 0; y < im.h; ++y) {
          for (x = 0; x < im.w; ++x) {
            for (c= 0; c < im.c; ++c) {
              float val = im.data[c * im.h * im.w + y * im.w + x];
              disp->imageData[y * step + x * im.c + c] = (unsigned char)(val * 255);
            }
          }
        }
        return disp;
      }

      static inline image mat_to_image(const cv::Mat& m) {
#if ROS_VERSION_MINOR < 15
        // ROS Melodic
        IplImage ipl = m;
#else
        // ROS Noetic
        IplImage ipl = cvIplImage(m);
#endif
        image im = ipl_to_image(&ipl);
        rgbgr_image(im);
        return im;
      }
  };
}
