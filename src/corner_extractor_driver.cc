// std
#include <memory>
#include <iostream>

// OpenCV
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

// Corner Extractor
#include "ros_cv_2d/program_options.h"
#include "ros_cv_2d/corner_extractor_driver.h"

using std::placeholders::_1;

CornerExtractorDriver::CornerExtractorDriver(const ProgramOptions& options)
: Node("corner_extractor"), show_corners_(options.show_corners), expose_debug_settings_(options.expose_debug_settings) {
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();
  img_subscriber_ = this->create_subscription<sensor_msgs::msg::Image> (
    options.input_img_topic, qos, std::bind(&CornerExtractorDriver::CornerCallback, this, _1));
  corner_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(options.output_img_topic, 1000);
  std::cout<<"Receiving images on topic "<<options.input_img_topic<<std::endl;
  std::cout<<"Publishing output on topic "<<options.output_img_topic<<std::endl;

  if (show_corners_) {
  CreateImageViewers();
  }
}

void CornerExtractorDriver::CreateImageViewers() {
  std::cout<<"Creating vis window..."<<std::endl;
  cv::namedWindow("corners");
  if (expose_debug_settings_) {
    std::cout<<"Creating trackbars..."<<std::endl;
    cv::createTrackbar("Max corners", "corners", &max_corners_, 200);
    cv::createTrackbar("Quality", "corners", &quality_level_, 100);
    cv::createTrackbar("Min dist", "corners", &min_dist_, 4000);
    cv::createTrackbar("free K param", "corners", &free_k_val_, 80);
    cv::createTrackbar("Use harris", "corners", &use_harris_, 1);
  }
}

void CornerExtractorDriver::CornerCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  src_ = cv_ptr->image;
  src_corners_overlay_ = src_.clone();
  cv::cvtColor(src_, src_gray_, cv::COLOR_BGR2GRAY);

  DoCornerExtraction(src_gray_);
  if (show_corners_) {
    ShowImages();
  }
}

void CornerExtractorDriver::ShowImages() {
  cv::imshow("corners", src_corners_overlay_);
  cv::waitKey(3);
}


void CornerExtractorDriver::DoCornerExtraction(const cv::Mat& img) {
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img,
                          corners,
                          std::max(max_corners_, 1),
                          std::max(static_cast<double>(quality_level_/1000.0), 0.01),
                          std::max(static_cast<double>(min_dist_/1000.0), 1.0),
                          cv::Mat(),
                          kBlockSize,
                          kGradientSize,
                          static_cast<bool>(use_harris_),
                          std::max(static_cast<double>(free_k_val_/1000.0), 0.01));


  for( size_t i = 0; i < corners.size(); i++ ) {
    cv::circle(src_corners_overlay_, corners[i], kCircleRadius, cv::Scalar(0, 255, 0), cv::FILLED );
  }
}
