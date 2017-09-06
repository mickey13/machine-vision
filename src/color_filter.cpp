#include <machine_vision/color_filter.h>

#include <ros/ros.h>

ColorFilter::ColorFilter(unsigned int minHue, unsigned int minSaturation, unsigned int minValue, unsigned int maxHue, unsigned int maxSaturation, unsigned int maxValue) {
  this->mMinColor = cv::Scalar(minHue, minSaturation, minValue);
  this->mMaxColor = cv::Scalar(maxHue, maxSaturation, maxValue);
}

cv::Mat ColorFilter::process(const cv::Mat& imageFrame) const {
  cv::Mat thresholdFrame;
  try {
    cv::Mat bgrImage = imageFrame.clone();
    cv::medianBlur(bgrImage, bgrImage, 3);
    cv::Mat hsvFrame;
    cv::cvtColor(bgrImage, hsvFrame, cv::COLOR_BGR2HSV);
    cv::Mat thresholdFrame;
    cv::inRange(hsvFrame, this->mMinColor, this->mMaxColor, thresholdFrame);
    return thresholdFrame.clone();
  }
  catch (cv::Exception exception) {
    ROS_ERROR("ColorFilter::process - %s", exception.what());
  }
}
