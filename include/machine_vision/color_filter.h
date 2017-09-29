#ifndef COLOR_FILTER_H
#define COLOR_FILTER_H

#include <cv_bridge/cv_bridge.h>

class ColorFilter {
public:
  ColorFilter(unsigned int minHue, unsigned int minSaturation, unsigned int minValue, unsigned int maxHue, unsigned int maxSaturation, unsigned int maxValue);
  cv::Mat filterImage(const cv::Mat& imageFrame) const;
  void test() const;

private:
  ColorFilter();

  cv::Scalar mMinColor;
  cv::Scalar mMaxColor;
};

#endif
