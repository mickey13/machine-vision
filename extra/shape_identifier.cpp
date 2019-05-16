#include <machine_vision/shape_identifier.h>

#include <string>

ShapeIdentifier::ShapeIdentifier() {

}

void ShapeIdentifier::processContour(const std::vector<cv::Point>& contour) const {
  std::vector<cv::Point> approximate;
  double perimeter = cv::arcLength(contour, true);
  cv::approxPolyDP(contour, approximate, 0.04 * perimeter, true);
}

cv::Mat ShapeIdentifier::findCircles(const cv::Mat& imageFrame, const cv::Mat& thresholdFrame, std::vector<cv::Vec3f>& circles) const {
  cv::Mat img = imageFrame.clone();
  cv::Mat gray = thresholdFrame.clone();
  cv::medianBlur(gray, gray, 5);
  cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 2, gray.rows, 200, 100);
  for (size_t i = 0; i < circles.size(); ++i) {
    cv::Vec3i c = circles[i];
    cv::circle(img, cv::Point(c[0], c[1]), c[2], cv::Scalar(0,0,255), 3, cv::LINE_AA);
    cv::circle(img, cv::Point(c[0], c[1]), 2, cv::Scalar(0,255,0), 3, cv::LINE_AA);
  }
  return img;
}
