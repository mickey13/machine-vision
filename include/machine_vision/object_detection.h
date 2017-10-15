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
  cv::Mat identifyTarget(const cv::Mat& imageFrame, const std::vector<std::vector<cv::Point>>& contours) const;
  void publishObservations(std::vector<std::vector<cv::Point>>& contours) const;
  void publishGeometryObservations(const std::vector<cv::Vec3f>& circles) const;
  cv::Mat averageObservationsAndPublish(const cv::Mat& imageFrame, std::vector<std::vector<cv::Point>>& contours) const;

  ros::NodeHandle* mRosNode;
  ros::Publisher mObservationPublisher;

  VideoHandler* mVideoHandler;
  std::map<std::string, ColorFilter> mColorFilters;
  std::map<std::string, ObjectType> mObjectTypes;
};

#endif
