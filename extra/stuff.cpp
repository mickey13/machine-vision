// This approach seems to not work very well for all but the most simple examples.
cv::Mat ObjectDetection::detectExteriorContours(const cv::Mat& imageFrame, bool isMono, std::vector<std::vector<cv::Point>>& contours) const {
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
  cv::Mat contourImage = cv::Mat::zeros(cannyOutput.size(), CV_8UC3);
  for (int i = 0; i < contours.size(); ++i) {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    cv::drawContours(contourImage, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
  }
  return contourImage;
}

cv::Mat ObjectDetection::publishObservations(const cv::Mat& imageFrame, std::vector<std::vector<cv::Point>>& contours) const {
  cv::Mat img = imageFrame.clone();
  machine_vision::ObservationArray observationArrayMsg;
  for (std::vector<std::vector<cv::Point>>::const_iterator iter = contours.begin(); iter != contours.end(); ++iter) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(*iter, center, radius);
    machine_vision::Observation observationMsg;
    observationMsg.position.x = this->mVideoHandler->normalizeWidthPosition(center.x);
    observationMsg.position.y = -1.0 * this->mVideoHandler->normalizeHeightPosition(center.y);
    observationMsg.radius = radius;
    observationMsg.geometry_detected = false;
    observationArrayMsg.observations.push_back(observationMsg);
    cv::circle(img, center, 2 * this->mVideoHandler->getLineThickness(), cv::Scalar(0,255,0), -1, cv::LINE_AA);
  }
  this->mObservationPublisher.publish(observationArrayMsg);
  return img;
}

cv::Mat ObjectDetection::filterByColor(const cv::Mat& imageFrame) const {
  cv::Mat thresholdFrame;
  if (imageFrame.dims > 0) {
    std::map<std::string, ColorFilter>::const_iterator iter = this->mColorFilters.find("white");
    if (iter != this->mColorFilters.end()) {
      thresholdFrame = iter->second.filterImage(imageFrame);
    }
    else {
      thresholdFrame = cv::Mat::zeros(imageFrame.size(), CV_8UC1);
    }
  }
  return thresholdFrame;
}
