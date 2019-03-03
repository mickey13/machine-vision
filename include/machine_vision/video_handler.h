#ifndef VIDEO_HANDLER_H
#define VIDEO_HANDLER_H

#include <machine_vision/distortion_corrector.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

class ObjectDetection;

class VideoHandler {
public:
  VideoHandler(
    ros::NodeHandle& rosNode,
    ros::Rate rosRate,
    std::string cameraTopic,
    unsigned int pixelWidth,
    unsigned int pixelHeight
  );
  ~VideoHandler();
  void configureDistortionCorrection(std::string calibrationFile);
  void registerCallback(ObjectDetection* ObjectDetection, void (ObjectDetection::*imageCallback)(cv::Mat imageFrame));
  void publishAnnotatedImage(const cv::Mat& imageFrame, std::string imageEncoding) const;
  void publishDebugImage(const cv::Mat& imageFrame, std::string imageEncoding) const;
  int getLineThickness() const;
  double normalizeWidthPosition(unsigned int x) const;
  double normalizeHeightPosition(unsigned int y) const;

private:
  VideoHandler();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  double normalizePosition(unsigned int position, unsigned int range) const;
  cv::Mat cullImage(const cv::Mat& imageFrame) const;

  const unsigned int PIXEL_WIDTH;
  const unsigned int PIXEL_HEIGHT;

  unsigned int mMarginWidth;
  unsigned int mMarginHeight;

  ros::NodeHandle* mRosNode;
  ros::Rate mRosRate;
  image_transport::ImageTransport mImageTransport;
  image_transport::Subscriber mCameraSubscriber;
  image_transport::Publisher mAnnotatedImagePublisher;
  image_transport::Publisher mFilteredImagePublisher;
  cv_bridge::CvImagePtr mCvImagePtr;
  DistortionCorrector* mDistortionCorrector;
  ObjectDetection* mObjectDetection;
  void (ObjectDetection::*mImageCallback)(cv::Mat imageFrame);
};

#endif
