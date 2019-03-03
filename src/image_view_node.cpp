#include <machine_vision/image_view.h>

#include <ros/ros.h>

#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_view");
  ros::NodeHandle rosNode;
  ros::Rate rosRate(10.0);

  ImageView imageView(rosNode, "image");

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
