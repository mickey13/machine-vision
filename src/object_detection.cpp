#include <machine_vision/object_detection.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

ObjectDetection::ObjectDetection(VideoHandler& videoHandler) {
  this->mVideoHandler = &videoHandler;
  this->mVideoHandler->registerCallback(this, &ObjectDetection::imageEvent);
}

void ObjectDetection::imageEvent(cv::Mat imageFrame) {
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat thresholdFrame = this->filterByColor(imageFrame);
  cv::Mat contoursFrame = this->detectExteriorContours(thresholdFrame, true, contours);
  cv::Mat annotatedFrame = this->identifyTarget(imageFrame, contours);
  this->mVideoHandler->publishAnnotatedImage(annotatedFrame, sensor_msgs::image_encodings::BGR8);
  this->mVideoHandler->publishDebugImage(contoursFrame, sensor_msgs::image_encodings::BGR8);
}

void ObjectDetection::loadColorFilters(std::string filename) {
  try {
    std::ifstream file(filename);
    std::string line;
    if (file.is_open()) {
      while (std::getline(file, line)) {
        this->configureColorFilter(line);
      }
      file.close();
    }
  }
  catch (std::exception exception) {
    ROS_ERROR("ObjectDetection::loadColorFilters - %s", exception.what());
  }
}

void ObjectDetection::configureColorFilter(std::string csvText) {
  std::stringstream ss(csvText);
  std::vector<unsigned int> params;
  std::string colorText;
  std::string token;
  getline(ss, colorText, ',');
  while (ss.good()) {
    getline(ss, token, ',');
    params.push_back(std::stoi(token));
  }
  ColorFilter colorFilter(params[0], params[1], params[2], params[3], params[4], params[5]);
  this->mColorFilters.insert(std::pair<std::string, ColorFilter>(colorText, colorFilter));
}

cv::Mat ObjectDetection::filterByColor(const cv::Mat& imageFrame) const {
  cv::Mat thresholdFrame;
  if (imageFrame.dims > 0) {
    std::map<std::string, ColorFilter>::const_iterator iter = this->mColorFilters.find("white");
    if (iter != this->mColorFilters.end()) {
      thresholdFrame = iter->second.process(imageFrame);
    }
  }
  return thresholdFrame;
}

cv::Mat ObjectDetection::detectExteriorContours(const cv::Mat& imageFrame, bool isMono, std::vector<std::vector<cv::Point>>& contours) const {
  // std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat grayImage;
  cv::Mat cannyOutput;
  cv::RNG rng(12345);
  int threshold = 100;
  if (isMono) {
    grayImage = imageFrame;
  }
  else {
    cv::cvtColor(imageFrame, grayImage, CV_BGR2GRAY);
  }
  cv::blur(grayImage, grayImage, cv::Size(3, 3));
  cv::Canny(grayImage, cannyOutput, threshold, threshold * 3, 3);
  cv::findContours(cannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  // cv::findContours(cannyOutput, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  cv::Mat contourImage = cv::Mat::zeros(cannyOutput.size(), CV_8UC3);
  for (int i = 0; i < contours.size(); ++i) {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    cv::drawContours(contourImage, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
  }
  return contourImage;
}

cv::Mat ObjectDetection::identifyTarget(const cv::Mat& imageFrame, const std::vector<std::vector<cv::Point>>& contours) const {
  cv::Mat targetImage = imageFrame.clone();
  cv::RNG rng(12345);
  int lineThickness = this->mVideoHandler->getLineThickness();
  for (int i = 0; i < contours.size(); ++i) {
    // if (ObjectDetection::isContourSquare(contours[i])) {
      cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
      cv::drawContours(targetImage, contours, i, color, lineThickness, 8);
    // }
  }
  return targetImage;
}
