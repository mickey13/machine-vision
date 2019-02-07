#include <machine_vision/image_view.h>

#include <ros/ros.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_view");
  ros::NodeHandle rosNode;
  ros::Rate rosRate(10.0);
  std::string cameraTopic = "image_raw";

  rosNode.param("machine_vision/camera_topic", cameraTopic, cameraTopic);
  std::cout << "Camera Topic: " << cameraTopic << std::endl;

  ImageView imageView(rosNode, cameraTopic);

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
