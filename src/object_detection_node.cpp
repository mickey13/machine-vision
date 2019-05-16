#include <machine_vision/object_detection.h>
#include <machine_vision/video_handler.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <string>

void loadColorFilters(const ros::NodeHandle& rosNode, ObjectDetection& objectDetection);
void loadObjectTypes(const ros::NodeHandle& rosNode, ObjectDetection& objectDetection);

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle rosNode;
  std::string cameraTopic = "image_raw";
  std::string ocamCalibrationFile;
  float frequency = 5.0;
  int pixelWidth = 640;
  int pixelHeight = 480;
  bool isEnabled = true;

  rosNode.param(ros::this_node::getNamespace() + "/frequency", frequency, frequency);
  rosNode.param(ros::this_node::getNamespace() + "/camera_topic", cameraTopic, cameraTopic);
  rosNode.param(ros::this_node::getNamespace() + "/pixel_width", pixelWidth, pixelWidth);
  rosNode.param(ros::this_node::getNamespace() + "/pixel_height", pixelHeight, pixelHeight);
  rosNode.param(ros::this_node::getNamespace() + "/enabled", isEnabled, isEnabled);
  rosNode.param(ros::this_node::getName() + "/enabled", isEnabled, isEnabled);

  ros::Rate rosRate(frequency);
  VideoHandler videoHandler(rosNode, rosRate, cameraTopic, pixelWidth, pixelHeight);
  if (rosNode.param(ros::this_node::getNamespace() + "/ocam_calibration_file", ocamCalibrationFile, ocamCalibrationFile)) {
    videoHandler.configureDistortionCorrection(ocamCalibrationFile);
  }
  ObjectDetection objectDetection(rosNode, videoHandler, isEnabled);
  XmlRpc::XmlRpcValue colorFilterStruct;
  XmlRpc::XmlRpcValue objectTypeStruct;
  rosNode.getParam(ros::this_node::getNamespace() + "/machine_vision/color_filters", colorFilterStruct);
  rosNode.getParam(ros::this_node::getNamespace() + "/machine_vision/object_types", objectTypeStruct);
  objectDetection.loadColorFilters(colorFilterStruct);
  objectDetection.loadObjectTypes(objectTypeStruct);

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
