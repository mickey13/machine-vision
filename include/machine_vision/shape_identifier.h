#ifndef SHAPE_IDENTIFIER_H
#define SHAPE_IDENTIFIER_H

#include <cv_bridge/cv_bridge.h>
#include <vector>

class ShapeIdentifier {
public:
  ShapeIdentifier();
  void processContour(const std::vector<cv::Point>& contour) const;
  cv::Mat findCircles(const cv::Mat& imageFrame, const cv::Mat& thresholdFrame, std::vector<cv::Vec3f>& circles) const;
};

#endif
