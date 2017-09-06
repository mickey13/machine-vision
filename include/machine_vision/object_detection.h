#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <machine_vision/video_handler.h>
#include <machine_vision/color_filter.h>

#include <map>
#include <string>

class ObjectDetection {
public:
  ObjectDetection(VideoHandler& videoHandler);
  void imageEvent(cv::Mat imageFrame);
  void loadColorFilters(std::string filename);

private:
  ObjectDetection();
  void configureColorFilter(std::string csvText);
  cv::Mat filterByColor(const cv::Mat& imageFrame) const;
  cv::Mat detectExteriorContours(const cv::Mat& imageFrame, bool isMono, std::vector<std::vector<cv::Point>>& contours) const;
  cv::Mat identifyTarget(const cv::Mat& imageFrame, const std::vector<std::vector<cv::Point>>& contours) const;

  VideoHandler* mVideoHandler;
  std::map<std::string, ColorFilter> mColorFilters;
};

#endif
