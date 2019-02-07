#include <machine_vision/image_view.h>

#include <opencv2/highgui/highgui.hpp>

static const std::string WINDOW_NAME = "Machine Vision";

ImageView::ImageView(ros::NodeHandle& rosNode, std::string cameraTopic) : mImageTransport(rosNode) {
  this->mRosNode = &rosNode;
  this->mCvImagePtr = NULL;
  this->mCameraSubscriber = this->mImageTransport.subscribe(cameraTopic, 1, &ImageView::imageCallback, this);
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
}

void ImageView::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  this->mCvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::imshow(WINDOW_NAME, this->mCvImagePtr->image.clone());
  cv::waitKey(1);
}
