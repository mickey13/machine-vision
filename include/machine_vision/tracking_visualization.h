#ifndef TRACKING_VISUALIZATION_H
#define TRACKING_VISUALIZATION_H

#include <machine_vision/TargetEstimateArray.h>

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <string>

class TrackingVisualization {
public:
  TrackingVisualization(ros::NodeHandle& rosNode);

private:
  TrackingVisualization();
  void targetEstimateCallback(const machine_vision::TargetEstimateArray::ConstPtr& msg);
  void drawTargetEstimates(cv::Mat image, std::vector<geometry_msgs::PoseWithCovariance> targetEstimates) const;
  void drawGrid(cv::Mat image) const;
  int getTextOffsetFromText(const std::string& text) const;
  cv::Mat convertCovarianceToPixelUnit(const boost::array<double, 36>& covariance) const;
  cv::RotatedRect getErrorEllipse(double chiSquareVal, cv::Point2i mean, cv::Mat covariance) const;

  ros::NodeHandle* mRosNode;
  ros::Subscriber mTargetEstimateSubscriber;
};

#endif
