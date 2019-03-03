#include <machine_vision/object_detection.h>
#include <machine_vision/shape_identifier.h>
#include <machine_vision/Observation.h>
#include <machine_vision/ObservationArray.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

ObjectDetection::ObjectDetection(
  ros::NodeHandle& rosNode,
  VideoHandler& videoHandler
) {
  this->mRosNode = &rosNode;
  this->mVideoHandler = &videoHandler;
  this->mVideoHandler->registerCallback(this, &ObjectDetection::imageEvent);
  this->mObservationPublisher = this->mRosNode->advertise<machine_vision::ObservationArray>("machine_vision/observations", 1);
}

void ObjectDetection::imageEvent(cv::Mat imageFrame) {
  ShapeIdentifier shapeIdentifier;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec3f> circles;
  cv::Mat thresholdFrame = this->filterObjectTypes(imageFrame);
  cv::Mat contoursFrame = this->detectExteriorContours(thresholdFrame, true, contours);
  cv::Mat annotatedFrame = shapeIdentifier.findCircles(imageFrame, thresholdFrame, circles);
  annotatedFrame = this->publishObservations(imageFrame, contours);
  this->mVideoHandler->publishAnnotatedImage(annotatedFrame, sensor_msgs::image_encodings::BGR8);
  this->mVideoHandler->publishDebugImage(thresholdFrame, sensor_msgs::image_encodings::MONO8);
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

cv::Mat ObjectDetection::filterObjectTypes(const cv::Mat& imageFrame) const {
  cv::Mat combinedFrame = cv::Mat::zeros(imageFrame.size(), CV_8UC1);
  for (std::map<std::string, ObjectType>::const_iterator iter = this->mObjectTypes.begin(); iter != this->mObjectTypes.end(); ++iter) {
    cv::Mat thresholdFrame = iter->second.filterImage(imageFrame);
    cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);
  }
  return combinedFrame;
}

cv::Mat ObjectDetection::filterByColor(const cv::Mat& imageFrame) const {
  cv::Mat thresholdFrame;
  if (imageFrame.dims > 0) {
    std::map<std::string, ColorFilter>::const_iterator iter = this->mColorFilters.find("white");
    if (iter != this->mColorFilters.end()) {
      thresholdFrame = iter->second.filterImage(imageFrame);
    }
    else {
      thresholdFrame = cv::Mat::zeros(imageFrame.size(), CV_8UC1);
    }
  }
  return thresholdFrame;
}

cv::Mat ObjectDetection::detectExteriorContours(const cv::Mat& imageFrame, bool isMono, std::vector<std::vector<cv::Point>>& contours) const {
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat grayImage;
  cv::Mat cannyOutput;
  cv::RNG rng(12345);
  int threshold = 100;
  if (isMono) {
    grayImage = imageFrame;
  }
  else {
    cv::cvtColor(imageFrame, grayImage, CV_BGR2GRAY);
  }
  cv::blur(grayImage, grayImage, cv::Size(3, 3));
  cv::Canny(grayImage, cannyOutput, threshold, threshold * 3, 3);
  cv::findContours(cannyOutput, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  cv::Mat contourImage = cv::Mat::zeros(cannyOutput.size(), CV_8UC3);
  for (int i = 0; i < contours.size(); ++i) {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    cv::drawContours(contourImage, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
  }
  return contourImage;
}

// cv::Mat ObjectDetection::identifyTarget(const cv::Mat& imageFrame, const std::vector<std::vector<cv::Point>>& contours) const {
//   cv::Mat targetImage = imageFrame.clone();
//   cv::RNG rng(12345);
//   int lineThickness = this->mVideoHandler->getLineThickness();
//   for (int i = 0; i < contours.size(); ++i) {
//     cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
//     cv::drawContours(targetImage, contours, i, color, lineThickness, 8);
//   }
//   return targetImage;
// }

// void ObjectDetection::publishObservations(std::vector<std::vector<cv::Point>>& contours) const {
//   machine_vision::ObservationArray observationArrayMsg;
//   for (std::vector<std::vector<cv::Point>>::const_iterator iter = contours.begin(); iter != contours.end(); ++iter) {
//     machine_vision::Observation observationMsg;
//     cv::Point2f center;
//     float radius;
//     cv::minEnclosingCircle(*iter, center, radius);
//     observationMsg.position.x = this->mVideoHandler->normalizeWidthPosition(center.x);
//     observationMsg.position.y = -1.0 * this->mVideoHandler->normalizeHeightPosition(center.y);
//     observationMsg.geometry_detected = false;
//     observationArrayMsg.observations.push_back(observationMsg);
//     ShapeIdentifier shapeIdentifier;
//     shapeIdentifier.processContour(*iter);
//   }
//   this->mObservationPublisher.publish(observationArrayMsg);
// }

// void ObjectDetection::publishGeometryObservations(const std::vector<cv::Vec3f>& circles) const {
//   machine_vision::ObservationArray observationArrayMsg;
//   for (std::vector<cv::Vec3f>::const_iterator iter = circles.begin(); iter != circles.end(); ++iter) {
//     machine_vision::Observation observationMsg;
//     observationMsg.position.x = this->mVideoHandler->normalizeWidthPosition((*iter)[0]);
//     observationMsg.position.y = -1.0 * this->mVideoHandler->normalizeHeightPosition((*iter)[1]);
//     observationMsg.geometry_detected = true;
//     observationArrayMsg.observations.push_back(observationMsg);
//   }
//   this->mObservationPublisher.publish(observationArrayMsg);
// }

cv::Mat ObjectDetection::publishObservations(const cv::Mat& imageFrame, std::vector<std::vector<cv::Point>>& contours) const {
  cv::Mat img = imageFrame.clone();
  machine_vision::ObservationArray observationArrayMsg;
  for (std::vector<std::vector<cv::Point>>::const_iterator iter = contours.begin(); iter != contours.end(); ++iter) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(*iter, center, radius);
    machine_vision::Observation observationMsg;
    observationMsg.position.x = this->mVideoHandler->normalizeWidthPosition(center.x);
    observationMsg.position.y = -1.0 * this->mVideoHandler->normalizeHeightPosition(center.y);
    observationMsg.radius = radius;
    observationMsg.geometry_detected = false;
    observationArrayMsg.observations.push_back(observationMsg);
    cv::circle(img, center, 2 * this->mVideoHandler->getLineThickness(), cv::Scalar(0,255,0), -1, cv::LINE_AA);
  }
  this->mObservationPublisher.publish(observationArrayMsg);
  return img;
}
