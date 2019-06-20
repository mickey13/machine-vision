#include <machine_vision/object_detection.h>
#include <machine_vision/Observation.h>
#include <machine_vision/ObservationArray.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

ObjectDetection::ObjectDetection(ros::NodeHandle &rosNode, VideoHandler &videoHandler, bool enabled) {
  this->mRosNode = &rosNode;
  this->mVideoHandler = &videoHandler;
  this->mVideoHandler->registerCallback(this, &ObjectDetection::imageEvent);
  this->mEnableService = this->mRosNode->advertiseService("machine_vision/enable", &ObjectDetection::enableService, this);
  this->mObservationPublisher = this->mRosNode->advertise<machine_vision::ObservationArray>("machine_vision/observations", 1);
  this->mDetectionEnabled = true;
}

void ObjectDetection::imageEvent(cv::Mat imageFrame) {
  if (this->mDetectionEnabled) {
    std::vector<std::vector<cv::Point>> contours;
    cv::Rect boundingBox;
    cv::Mat thresholdFrame = this->filterObjectTypes(imageFrame);
    cv::Mat annotatedFrame = this->detectBoundingBox(imageFrame, thresholdFrame, boundingBox);
    this->publishObservation(boundingBox);
    this->mVideoHandler->publishAnnotatedImage(annotatedFrame, sensor_msgs::image_encodings::BGR8);
    this->mVideoHandler->publishDebugImage(thresholdFrame, sensor_msgs::image_encodings::MONO8);
  }
}

void ObjectDetection::loadColorFilters(XmlRpc::XmlRpcValue& colorFilterStruct) {
  ROS_ASSERT(colorFilterStruct.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < colorFilterStruct.size(); ++i) {
    ROS_ASSERT(colorFilterStruct[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::iterator iter = colorFilterStruct[i].begin(); iter != colorFilterStruct[i].end(); ++iter) {
      ROS_ASSERT(iter->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(iter->second.hasMember("min_hue"));
      ROS_ASSERT(iter->second.hasMember("min_saturation"));
      ROS_ASSERT(iter->second.hasMember("min_value"));
      ROS_ASSERT(iter->second.hasMember("max_hue"));
      ROS_ASSERT(iter->second.hasMember("max_saturation"));
      ROS_ASSERT(iter->second.hasMember("max_value"));
      ColorFilter colorFilter(
        static_cast<int>(iter->second["min_hue"]),
        static_cast<int>(iter->second["min_saturation"]),
        static_cast<int>(iter->second["min_value"]),
        static_cast<int>(iter->second["max_hue"]),
        static_cast<int>(iter->second["max_saturation"]),
        static_cast<int>(iter->second["max_value"])
      );
      this->mColorFilters.insert(std::pair<std::string, ColorFilter>(iter->first, colorFilter));
    }
  }
}

void ObjectDetection::loadObjectTypes(XmlRpc::XmlRpcValue& objectTypeStruct) {
  ROS_ASSERT(objectTypeStruct.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < objectTypeStruct.size(); ++i)  {
    ROS_ASSERT(objectTypeStruct[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::ValueStruct::iterator iter = objectTypeStruct[i].begin(); iter != objectTypeStruct[i].end(); ++iter) {
      ROS_ASSERT(iter->second.getType() == XmlRpc::XmlRpcValue::TypeStruct);
      ROS_ASSERT(iter->second.hasMember("name"));
      ROS_ASSERT(iter->second.hasMember("color_filters"));
      ROS_ASSERT(iter->second["color_filters"].getType() == XmlRpc::XmlRpcValue::TypeArray);
      std::vector<ColorFilter*> colorFilters;
      for (int i = 0; i < iter->second["color_filters"].size(); ++i) {
        std::string filterName = static_cast<std::string>((iter->second)["color_filters"][i]);
        colorFilters.push_back(&(this->mColorFilters.find(filterName)->second));
      }
      ObjectType objectType(iter->second["name"], colorFilters);
      this->mObjectTypes.insert(std::pair<std::string, ObjectType>(iter->first, objectType));
    }
  }
}

bool ObjectDetection::enableService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  if (this->mDetectionEnabled == request.data) {
    response.success = false;
    response.message = "Object detection has already been set to [" + this->enabledStateString() + "].";
  }
  else {
    this->mDetectionEnabled = request.data;
    response.success = true;
    response.message = "Object detection is now set to [" + this->enabledStateString() + "].";
  }
  return true;
}

cv::Mat ObjectDetection::filterObjectTypes(const cv::Mat& imageFrame) const {
  cv::Mat combinedFrame = cv::Mat::zeros(imageFrame.size(), CV_8UC1);
  for (std::map<std::string, ObjectType>::const_iterator iter = this->mObjectTypes.begin(); iter != this->mObjectTypes.end(); ++iter) {
    cv::Mat thresholdFrame = iter->second.filterImage(imageFrame);
    cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);
  }
  return combinedFrame;
}

cv::Mat ObjectDetection::detectBoundingBox(const cv::Mat& imageFrame, const cv::Mat& threshold, cv::Rect& boundingBox) const {
  cv::Mat pixels;
  cv::Mat annotatedFrame = imageFrame.clone();
  cv::findNonZero(threshold, pixels);
  boundingBox = cv::boundingRect(pixels);
  if (boundingBox.area() > 0) {
    cv::Point2i center(boundingBox.x + (boundingBox.width / 2), boundingBox.y + (boundingBox.height / 2));
    cv::rectangle(annotatedFrame, boundingBox, cv::Scalar(55, 140, 167), 2);
    cv::circle(annotatedFrame, center, this->computeMarkerSize(boundingBox), cv::Scalar(232, 119, 34), -1, cv::LINE_AA);
  }
  return annotatedFrame;
}

void ObjectDetection::publishObservation(const cv::Rect& boundingBox) const {
  machine_vision::ObservationArray observationArrayMsg;
  if (boundingBox.area() > 0) {
    machine_vision::Observation observationMsg;
    observationMsg.position.x = this->mVideoHandler->normalizeWidthPosition(boundingBox.x + (boundingBox.width / 2));
    observationMsg.position.y = -1.0 * this->mVideoHandler->normalizeHeightPosition(boundingBox.y + (boundingBox.height / 2));
    observationMsg.area = boundingBox.area();
    observationArrayMsg.observations.push_back(observationMsg);
  }
  this->mObservationPublisher.publish(observationArrayMsg);
}

int ObjectDetection::computeMarkerSize(const cv::Rect& boundingBox) const {
  const int MIN_MARKER_SIZE = 5;
  int largestSide = boundingBox.width > boundingBox.height ? boundingBox.width : boundingBox.height;
  int markerSize = (int)(0.025 * largestSide);
  return markerSize > MIN_MARKER_SIZE ? markerSize : MIN_MARKER_SIZE;
}

std::string ObjectDetection::enabledStateString() const {
  return this->mDetectionEnabled ? "enabled" : "disabled";
}
