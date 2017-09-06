#include <machine_vision/object_detection.h>
#include <machine_vision/video_handler.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle rosNode;
  std::string cameraTopic = "camera/image_raw";
  std::string lensCalibrationFile = "config/fisheye_calibration.txt";
  bool correctDistortion = false;
  float frequency = 5.0;
  int pixelWidth = 640;
  int pixelHeight = 480;

  rosNode.param(ros::this_node::getName() + "/frequency", frequency, frequency);
  rosNode.param(ros::this_node::getName() + "/camera_topic", cameraTopic, cameraTopic);
  rosNode.param(ros::this_node::getName() + "/pixel_width", pixelWidth, pixelWidth);
  rosNode.param(ros::this_node::getName() + "/pixel_height", pixelHeight, pixelHeight);
  rosNode.param(ros::this_node::getName() + "/correct_distortion", correctDistortion, correctDistortion);
  rosNode.param(ros::this_node::getName() + "/lens_calibration", lensCalibrationFile, lensCalibrationFile);

  ros::Rate rosRate(frequency);
  VideoHandler videoHandler(rosNode, rosRate, cameraTopic, pixelWidth, pixelHeight);
  if (correctDistortion) {
    videoHandler.configureDistortionCorrection(lensCalibrationFile);
  }
  ObjectDetection objectDetection(videoHandler);
  objectDetection.loadColorFilters(ros::package::getPath("machine_vision") + "/config/color_filters.csv");
  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
