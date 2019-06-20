#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H

#include <machine_vision/video_handler.h>
#include <machine_vision/color_filter.h>
#include <machine_vision/object_type.h>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <map>
#include <string>

class ObjectDetection {
public:
  ObjectDetection(ros::NodeHandle &rosNode, VideoHandler &videoHandler, bool enabled);
  void imageEvent(cv::Mat imageFrame);
  void loadColorFilters(XmlRpc::XmlRpcValue& colorFilterStruct);
  void loadObjectTypes(XmlRpc::XmlRpcValue& objectTypeStruct);

private:
  ObjectDetection();
  bool enableService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  cv::Mat filterObjectTypes(const cv::Mat& imageFrame) const;
  cv::Mat detectBoundingBox(const cv::Mat& imageFrame, const cv::Mat& threshold, cv::Rect& boundingBox) const;
  void publishObservation(const cv::Rect& boundingBox) const;
  int computeMarkerSize(const cv::Rect& boundingBox) const;
  std::string enabledStateString() const;

  ros::NodeHandle* mRosNode;
  ros::ServiceServer mEnableService;
  ros::Publisher mObservationPublisher;
  VideoHandler* mVideoHandler;
  std::map<std::string, ColorFilter> mColorFilters;
  std::map<std::string, ObjectType> mObjectTypes;
  std::atomic<bool> mDetectionEnabled;
};

#endif
