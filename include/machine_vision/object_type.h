#ifndef OBJECT_DEFINITION_H
#define OBJECT_DEFINITION_H

#include <machine_vision/color_filter.h>
#include <machine_vision/roi_marker.h>

#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <string>

class ObjectType {
public:
  ObjectType(std::string name, const std::vector<ColorFilter>& colorFilters, const std::vector<RoiMarker>& roiMarkers);
  cv::Mat filterImage(const cv::Mat& imageFrame) const;
  void debug() const;

private:
  ObjectType();

  std::string mName;
  std::vector<ColorFilter> mColorFilters;
  std::vector<RoiMarker> mRoiMarkers;
};

#endif
