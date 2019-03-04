#include <machine_vision/tracking_visualization.h>

#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tracking_visualization");
  ros::NodeHandle rosNode;
  ros::Rate rosRate(10.0);

  TrackingVisualization trackingVisualization(rosNode);

  while (rosNode.ok()) {
    ros::spinOnce();
    rosRate.sleep();
  }
  return 0;
}
