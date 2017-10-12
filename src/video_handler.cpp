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
  this->mObjectDetection = NULL;
  this->mImageCallback = NULL;
  this->mCameraSubscriber = this->mImageTransport.subscribe(cameraTopic.c_str(), QUEUE_SIZE, &VideoHandler::imageCallback, this);
  this->mCameraPublisher = this->mImageTransport.advertise(VideoHandler::renameTopic(cameraTopic, "_annotated").c_str(), QUEUE_SIZE);
  this->mDebugPublisher = this->mImageTransport.advertise(VideoHandler::renameTopic(cameraTopic, "_debug").c_str(), QUEUE_SIZE);
  this->mMarginWidth = 0;
  this->mMarginHeight = 0;
}

VideoHandler::~VideoHandler() {
  if (this->mDistortionCorrector != NULL) {
    delete this->mDistortionCorrector;
  }
}

void VideoHandler::configureDistortionCorrection(std::string calibrationFile) {
  this->mDistortionCorrector = new DistortionCorrector(calibrationFile, this->PIXEL_HEIGHT, this->PIXEL_WIDTH);
  this->mMarginWidth = PIXEL_WIDTH / 4;
  this->mMarginHeight = PIXEL_HEIGHT / 4;
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

double VideoHandler::normalizeWidthPosition(unsigned int x) const {
  return this->normalizePosition(x, this->PIXEL_WIDTH);
}

double VideoHandler::normalizeHeightPosition(unsigned int y) const {
  return this->normalizePosition(y, this->PIXEL_HEIGHT);
}

void VideoHandler::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  this->mCvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  if (this->mDistortionCorrector != NULL) {
    this->mCvImagePtr->image = this->cullImage(this->mDistortionCorrector->undistortImage(this->mCvImagePtr->image));
  }
  if (this->mObjectDetection != NULL && this->mImageCallback != NULL) {
    (this->mObjectDetection->*this->mImageCallback)(this->mCvImagePtr->image.clone());
  }
}

double VideoHandler::normalizePosition(unsigned int position, unsigned int range) const {
  return (((double) position) - ((double) (range / 2))) / ((double) (range / 2));
}

cv::Mat VideoHandler::cullImage(const cv::Mat& imageFrame) const {
  cv::Rect rectRoi = cv::Rect(this->mMarginWidth, this->mMarginHeight, PIXEL_WIDTH - (2 * this->mMarginWidth), PIXEL_HEIGHT - (2 * this->mMarginHeight));
  cv::Mat imageRoi = imageFrame(rectRoi);
  return imageRoi;
}

std::string VideoHandler::renameTopic(std::string topicName, std::string tag) {
  std::size_t pos = topicName.find("_raw");
  if (pos != std::string::npos) {
    topicName = topicName.substr(0, pos);
  }
  return topicName + tag;
}
