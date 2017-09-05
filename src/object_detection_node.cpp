#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle rosNode;
  float frequency = 10.0;
  rosNode.param(ros::this_node::getName() + "/frequency", frequency, frequency);
  ros::Rate rosRate(frequency);
  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
