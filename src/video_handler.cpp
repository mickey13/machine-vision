#include <machine_vision/video_handler.h>

static const int QUEUE_SIZE = 1;

VideoHandler::VideoHandler(
  ros::NodeHandle& rosNode,
  ros::Rate rosRate,
  std::string cameraTopic,
  unsigned int pixelWidth,
  unsigned int pixelHeight
) : mRosRate(rosRate),
    mImageTransport(rosNode),
    PIXEL_WIDTH(pixelWidth),
    PIXEL_HEIGHT(pixelHeight) {
  this->mRosNode = &rosNode;
  this->mCvImagePtr = NULL;
  this->mDistortionCorrector = NULL;
  this->mCameraSubscriber = this->mImageTransport.subscribe(cameraTopic.c_str(), QUEUE_SIZE, &VideoHandler::imageCallback, this);
  this->mCameraPublisher = this->mImageTransport.advertise(VideoHandler::renameTopic(cameraTopic, "_annotated").c_str(), QUEUE_SIZE);
  this->mDebugPublisher = this->mImageTransport.advertise(VideoHandler::renameTopic(cameraTopic, "_debug").c_str(), QUEUE_SIZE);
}

VideoHandler::~VideoHandler() {
  if (this->mDistortionCorrector != NULL) {
    delete this->mDistortionCorrector;
  }
}

void VideoHandler::configureDistortionCorrection(std::string calibrationFile) {
  this->mDistortionCorrector = new DistortionCorrector(calibrationFile, this->PIXEL_HEIGHT, this->PIXEL_WIDTH);
}

void VideoHandler::registerCallback(ObjectDetection* ObjectDetection, void (ObjectDetection::*imageCallback)(cv::Mat imageFrame)) {
  this->mObjectDetection = ObjectDetection;
  this->mImageCallback = imageCallback;
}

void VideoHandler::publishAnnotatedImage(const cv::Mat& imageFrame, std::string imageEncoding) const {
  cv_bridge::CvImage cvImage;
  cvImage.encoding = imageEncoding;
  cvImage.image = imageFrame;
  this->mCameraPublisher.publish(cvImage.toImageMsg());
}

void VideoHandler::publishDebugImage(const cv::Mat& imageFrame, std::string imageEncoding) const {
  cv_bridge::CvImage cvImage;
  cvImage.encoding = imageEncoding;
  cvImage.image = imageFrame;
  this->mDebugPublisher.publish(cvImage.toImageMsg());
}

int VideoHandler::getLineThickness() const {
  return this->PIXEL_WIDTH / 256;
}

void VideoHandler::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  this->mCvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (this->mDistortionCorrector != NULL) {
    this->mCvImagePtr->image = this->mDistortionCorrector->undistortImage(this->mCvImagePtr->image);
  }
  (*this->mObjectDetection.*this->mImageCallback)(this->mCvImagePtr->image.clone());
}

std::string VideoHandler::renameTopic(std::string topicName, std::string tag) {
  std::size_t pos = topicName.find("_raw");
  if (pos != std::string::npos) {
    topicName = topicName.substr(0, pos);
  }
  return topicName + tag;
}
