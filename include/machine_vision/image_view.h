#ifndef IMAGE_VIEW_H
#define IMAGE_VIEW_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>

class ImageView {
public:
  ImageView(ros::NodeHandle& rosNode, std::string cameraTopic);

private:
  ImageView();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle* mRosNode;
  image_transport::ImageTransport mImageTransport;
  image_transport::Subscriber mCameraSubscriber;
  cv_bridge::CvImagePtr mCvImagePtr;
};

#endif
