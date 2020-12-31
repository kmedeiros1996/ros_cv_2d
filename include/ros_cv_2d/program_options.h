#ifndef PROGRAM_OPTIONS_H
#define PROGRAM_OPTIONS_H

// std
#include <string>

// Third party
#include <cxxopts.hpp>

struct ProgramOptions {
  std::string input_img_topic{"/camera/color/image_raw"};
  std::string output_img_topic{"/img_corners"};
  bool show_corners{true};
  bool expose_debug_settings{true};
};

ProgramOptions ParseArgs(int argc, char** argv) {
  ProgramOptions program_options;
  cxxopts::Options options("Scan Matcher Node", "Match scans sequentially from one topic or from two separate topics.");
  options.add_options()
    ("i,in_img", "input topic to receive images on",
    cxxopts::value<std::string>(program_options.input_img_topic)->default_value("/camera/color/image_raw"))
    ("o,out_img", "output topic to publish images w/ extracted corners on",
    cxxopts::value<std::string>(program_options.output_img_topic)->default_value("/img_corners"))
    ("show_corners", "enable extracted corner visualization",
    cxxopts::value<bool>(program_options.show_corners))
    ("tweak", "expose corner parameters to sliders and allow modification in real time",
    cxxopts::value<bool>(program_options.expose_debug_settings))
    ("h,help", "Print usage info");

  auto result = options.parse(argc, argv);
  if (result.count("help")) {
    std::cout <<  options.help()  << std::endl;
    exit(0);
  }

  return program_options;
}

#endif //PROGRAM_OPTIONS_H
