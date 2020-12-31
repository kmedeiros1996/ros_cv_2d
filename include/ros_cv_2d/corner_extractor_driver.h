#ifndef CORNER_EXTRACTOR_DRIVER_H
#define CORNER_EXTRACTOR_DRIVER_H


// OpenCV
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// Corner Extractor
#include "ros_cv_2d/program_options.h"

constexpr int kBlockSize = 3;
constexpr int kGradientSize = 3;
constexpr int kMaxCorners = 20;
constexpr int kQualityLevel = 10;
constexpr int kMinDistance = 100;
constexpr int kVal = 40;
constexpr int kCircleRadius = 4;

class CornerExtractorDriver : public rclcpp::Node {
public:
  CornerExtractorDriver(const ProgramOptions& options);

private:
  void CornerCallback(const sensor_msgs::msg::Image::SharedPtr msg);


  void CreateImageViewers();
  void ShowImages();
  void DoCornerExtraction(const cv::Mat& img);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr corner_img_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscriber_;

  cv::Mat src_;
  cv::Mat src_gray_;
  cv::Mat src_corners_overlay_;

  int max_corners_{kMaxCorners};
  int quality_level_{kQualityLevel};
  int min_dist_{kMinDistance};
  int free_k_val_{kVal};
  int use_harris_ {0};

  bool show_corners_;
  bool expose_debug_settings_;
};


#endif //CORNER_EXTRACTOR_DRIVER_H
