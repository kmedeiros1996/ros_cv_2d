// std
#include <memory>
#include <iostream>

//ROS
#include "rclcpp/rclcpp.hpp"

//Corner Extractor
#include "ros_cv_2d/corner_extractor_driver.h"
#include "ros_cv_2d/program_options.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ProgramOptions opts = ParseArgs(argc, argv);
  rclcpp::spin(std::make_shared<CornerExtractorDriver>(opts));
  rclcpp::shutdown();
  std::cout<<"Shutting down..."<<std::endl;
  return 0;
}
