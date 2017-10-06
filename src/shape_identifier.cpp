#include <machine_vision/shape_identifier.h>

#include <string>

ShapeIdentifier::ShapeIdentifier() {

}

void ShapeIdentifier::processContour(const std::vector<cv::Point>& contour) const {
  std::vector<cv::Point> approximate;
  std::string shape = "poop";
  double perimeter = cv::arcLength(contour, true);
  cv::approxPolyDP(contour, approximate, 0.04 * perimeter, true);
}

cv::Mat ShapeIdentifier::findCircles(const cv::Mat& imageFrame, const cv::Mat& thresholdFrame) const {
  cv::Mat img = imageFrame.clone();
  return img;
}
