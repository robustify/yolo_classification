#include "YoloClassification.hpp"

using namespace cv;

namespace yolo_classification
{

  const char *COCO_CLASSES_[] = {"person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};

  YoloClassification::YoloClassification(ros::NodeHandle n, ros::NodeHandle pn)
  {
    pn.param("darknet_cfg_file", darknet_cfg_file_, std::string(""));
    pn.param("darknet_weights_file", darknet_weights_file_, std::string(""));

    std::string cam;
    pn.param("camera_name", cam, std::string("camera"));

    srv_.setCallback(boost::bind(&YoloClassification::reconfig, this, _1, _2));
    net_ = load_network(const_cast<char*>(darknet_cfg_file_.c_str()), const_cast<char*>(darknet_weights_file_.c_str()), 0);

    sub_image_ = ros::NodeHandle(cam).subscribe("image_rect_color", 1, &YoloClassification::recvImage, this);

    skip_ = 0;
    namedWindow("Output", WINDOW_NORMAL);
  }

  void YoloClassification::recvImage(const sensor_msgs::ImageConstPtr& msg)
  {
    // Skip frames to save computational bandwidth
    if (skip_ >= cfg_.skip) {
      skip_ = 0;
    } else {
      skip_++;
      return;
    }

    // Convert ROS image message into an OpenCV Mat
    Mat raw_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    
    // Run image through the neural network and return classifications and bounding boxes
    DarknetClassificationArray darknet_bboxes;
    runDarknet(raw_img, darknet_bboxes);

    // Visualize output
    for (auto& bbox : darknet_bboxes) {
      cv::Point2d corner(std::get<0>(bbox).x, std::get<0>(bbox).y);
      rectangle(raw_img, std::get<0>(bbox), Scalar(0, 255, 0));
      putText(raw_img, std::get<1>(bbox), corner, FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255));
    }

    imshow("Output", raw_img);
    waitKey(1);
  }

  void YoloClassification::runDarknet(const Mat& raw_img, DarknetClassificationArray& darknet_bboxes)
  {
    image temp_img = mat_to_image(raw_img);
    image im = resize_image(temp_img, net_->w, net_->h);
    free_image(temp_img);

    double image_scale_x = (double)raw_img.cols / (double)im.h;
    double image_scale_y = (double)raw_img.rows / (double)im.w;
    set_batch_network(net_, 1);
    network_predict(net_, im.data);
    int nboxes;
    detection *dets = get_network_boxes(net_, im.w, im.h, cfg_.thres, cfg_.hier, NULL, 0, &nboxes);
    free_image(im);

    for (int i = 0; i < nboxes; i++) {
      int best_classification = -1;
      double highest_prob = -INFINITY;
      for (int j = 0; j < dets[i].classes; j++) {
        double prob = dets[i].prob[j];
        if ((prob > cfg_.min_prob) && (prob > highest_prob)) {
          highest_prob = prob;
          best_classification = j;
        }
      }

      if (best_classification < 0) {
        continue;
      }

      box b = dets[i].bbox;
      int left  = (int)(image_scale_x * (b.x - 0.5 * b.w));
      int right = (int)(image_scale_x * (b.x + 0.5 * b.w));
      int top   = (int)(image_scale_y * (b.y - 0.5 * b.h));
      int bot   = (int)(image_scale_y * (b.y + 0.5 * b.h));

      std::tuple<Rect2d, std::string, double> candidate_bbox(Rect2d(left, top, right - left, bot - top),
                                                            std::string(COCO_CLASSES_[best_classification]),
                                                            highest_prob);

      bool found_duplicate = false;
      for (auto& bbox : darknet_bboxes) {
        int dx = std::abs(std::get<0>(bbox).x - left);
        int dy = std::abs(std::get<0>(bbox).y - top);
        if ((dx < cfg_.duplicate_thres) && (dy < cfg_.duplicate_thres)) {
          found_duplicate = true;
          break;
        }
      }

      if (!found_duplicate) {
        darknet_bboxes.push_back(candidate_bbox);
      }
    }
    free_detections(dets, nboxes);
  }

  void YoloClassification::reconfig(DarknetConfig& config, uint32_t level) {
    cfg_ = config;
  }

}