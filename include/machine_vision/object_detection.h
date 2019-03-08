#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <machine_vision/video_handler.h>
#include <machine_vision/color_filter.h>
#include <machine_vision/object_type.h>

#include <map>
#include <string>

class ObjectDetection {
public:
  ObjectDetection(
    ros::NodeHandle& rosNode,
    VideoHandler& videoHandler
  );
  void imageEvent(cv::Mat imageFrame);
  void loadColorFilters(XmlRpc::XmlRpcValue& colorFilterStruct);
  void loadObjectTypes(XmlRpc::XmlRpcValue& objectTypeStruct);

private:
  ObjectDetection();
  cv::Mat filterObjectTypes(const cv::Mat& imageFrame) const;
  cv::Mat filterByColor(const cv::Mat& imageFrame) const;
  cv::Mat detectExteriorContours(const cv::Mat& imageFrame, bool isMono, std::vector<std::vector<cv::Point>>& contours) const;
  cv::Mat detectBoundingBox(const cv::Mat& imageFrame, const cv::Mat& threshold, cv::Rect& boundingBox) const;
  // cv::Mat publishObservations(const cv::Mat& imageFrame, std::vector<std::vector<cv::Point>>& contours) const;
  void publishObservation(const cv::Rect& boundingBox) const;

  ros::NodeHandle* mRosNode;
  ros::Publisher mObservationPublisher;

  VideoHandler* mVideoHandler;
  std::map<std::string, ColorFilter> mColorFilters;
  std::map<std::string, ObjectType> mObjectTypes;
};

#endif
