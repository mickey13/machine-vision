#include <machine_vision/tracking_visualization.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

static const std::string WINDOW_NAME = "Tracking Visualization";
static const unsigned int DISPLAY_WIDTH = 1000;
static const unsigned int DISPLAY_HEIGHT = 1000;
static const unsigned int PIXELS_PER_METER = 100;
static const unsigned int TICK_SIZE = 8;
static const unsigned int MARKER_SIZE = 5;

TrackingVisualization::TrackingVisualization(ros::NodeHandle& rosNode) {
  this->mRosNode = &rosNode;
  this->mTargetEstimateSubscriber = this->mRosNode->subscribe<machine_vision::TargetEstimateArray>("perception/target_estimates", 1, &TrackingVisualization::targetEstimateCallback, this);
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
}

void TrackingVisualization::targetEstimateCallback(const machine_vision::TargetEstimateArray::ConstPtr& msg) {
  cv::Mat image(DISPLAY_HEIGHT, DISPLAY_WIDTH, CV_8UC3, cv::Scalar(50, 50, 50));
  this->drawGrid(image);
  this->drawTargetEstimates(image, msg->target_estimates);
  cv::imshow(WINDOW_NAME, image);
  cv::waitKey(1);
}

void TrackingVisualization::drawTargetEstimates(cv::Mat image, std::vector<machine_vision::TargetEstimate> targetEstimates) const {
  for (std::vector<machine_vision::TargetEstimate>::const_iterator iter = targetEstimates.begin(); iter != targetEstimates.end(); ++iter) {
    this->drawPredictionEstimate(image, iter->prediction);
    this->drawBeliefEstimate(image, iter->belief);
    if (iter->time_since_observation < 0.1) {
      this->drawObservation(image, iter->observation);
    }
  }
}

void TrackingVisualization::drawGrid(cv::Mat image) const {
  const int width = image.size().width;
  const int height = image.size().height;
  cv::line(image, cv::Point(0, height / 2), cv::Point(width, height / 2), cv::Scalar(255, 255, 255));
  cv::line(image, cv::Point(width / 2, 0), cv::Point(width / 2, height), cv::Scalar(255, 255, 255));
  int endTick = (int)(((DISPLAY_WIDTH - 10) / PIXELS_PER_METER) / 2);
  int startTick = -1 * endTick;
  for (int i = startTick; i <= endTick; ++i) {
    if (i != 0) {
      int offset = (i < 0) ? 17 : 5;
      cv::line(image, cv::Point((width / 2) + i * PIXELS_PER_METER, (height / 2) - TICK_SIZE), cv::Point((width / 2) + i * PIXELS_PER_METER, (height / 2) + TICK_SIZE), cv::Scalar(255, 255, 255));
      cv::putText(image, std::to_string(i), cv::Point((width / 2) - offset + i * PIXELS_PER_METER, (height / 2) + 3 * TICK_SIZE), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
    }
  }
  endTick = (int)(((DISPLAY_HEIGHT - 10) / PIXELS_PER_METER) / 2);
  startTick = -1 * endTick;
  for (int i = startTick; i <= endTick; ++i) {
    if (i != 0) {
      cv::line(image, cv::Point((width / 2) - TICK_SIZE, (height / 2) + i * PIXELS_PER_METER), cv::Point((width / 2) + TICK_SIZE, (height / 2) + i * PIXELS_PER_METER), cv::Scalar(255, 255, 255));
      cv::putText(image, std::to_string(-1 * i), cv::Point((width / 2) + 1.5 * TICK_SIZE, (height / 2) + 5 + i * PIXELS_PER_METER), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
    }
  }
  cv::putText(image, "East (m)", cv::Point(width - 74, (height / 2) + 2.5 * TICK_SIZE), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
  cv::putText(image, "North (m)", cv::Point((width / 2) + TICK_SIZE, 15), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
}

void TrackingVisualization::drawBeliefEstimate(cv::Mat image, geometry_msgs::PoseWithCovariance beliefEstimate) const {
  unsigned int x = (unsigned int)((image.size().width / 2) + (PIXELS_PER_METER * beliefEstimate.pose.position.x));
  unsigned int y = (unsigned int)((image.size().height / 2) - (PIXELS_PER_METER * beliefEstimate.pose.position.y));
  // Draw the mean
  cv::circle(image, cv::Point2i(x, y), MARKER_SIZE, cv::Scalar(47, 253, 41), 3);
  // Draw the covariance
  cv::Mat covarianceMatrix = this->convertCovarianceToPixelUnit(beliefEstimate.covariance);
  cv::RotatedRect ellipse = getErrorEllipse(2.4477, cv::Point2i(x, y), covarianceMatrix);
  cv::ellipse(image, ellipse, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  // Draw the coordinates of the mean
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2) << "(" << beliefEstimate.pose.position.x << ", " << beliefEstimate.pose.position.y << ")";
  int offset = this->getTextOffsetFromText(ss.str());
  cv::putText(image, ss.str(), cv::Point(x - offset, y + 20), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255, 255, 255));
}

void TrackingVisualization::drawPredictionEstimate(cv::Mat image, geometry_msgs::PoseWithCovariance predictionEstimate) const {
  unsigned int x = (unsigned int)((image.size().width / 2) + (PIXELS_PER_METER * predictionEstimate.pose.position.x));
  unsigned int y = (unsigned int)((image.size().height / 2) - (PIXELS_PER_METER * predictionEstimate.pose.position.y));
  // Draw the covariance
  cv::Mat covarianceMatrix = this->convertCovarianceToPixelUnit(predictionEstimate.covariance);
  cv::RotatedRect ellipse = getErrorEllipse(2.4477, cv::Point2i(x, y), covarianceMatrix);
  cv::ellipse(image, ellipse, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
}

void TrackingVisualization::drawObservation(cv::Mat image, geometry_msgs::Pose observation) const {
  unsigned int x = (unsigned int)((image.size().width / 2) + (PIXELS_PER_METER * observation.position.x));
  unsigned int y = (unsigned int)((image.size().height / 2) - (PIXELS_PER_METER * observation.position.y));
  cv::circle(image, cv::Point2i(x, y), 1, cv::Scalar(0, 0, 255), 3);
}

cv::Mat TrackingVisualization::convertCovarianceToPixelUnit(const boost::array<double, 36>& covariance) const {
  cv::Mat_<double> cov(2, 2);
  std::vector<unsigned int> indexes{ 0, 1, 6, 7 };
  unsigned int index = 0;
  for (std::vector<unsigned int>::const_iterator iter = indexes.begin(); iter != indexes.end(); ++iter) {
    cov(index / 2, index % 2) = PIXELS_PER_METER * covariance[*iter];
    index++;
  }
  return cov;
}

int TrackingVisualization::getTextOffsetFromText(const std::string& text) const {
  return 8 * (text.size() / 2);
}

cv::RotatedRect TrackingVisualization::getErrorEllipse(double chiSquareVal, cv::Point2i mean, cv::Mat covariance) const {
	cv::Mat eigenvalues;
  cv::Mat eigenvectors;
	cv::eigen(covariance, eigenvalues, eigenvectors);
	// Calculate the angle between the largest eigenvector and the x-axis
	double angle = atan2(eigenvectors.at<double>(0, 1), eigenvectors.at<double>(0, 0));
	// Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
	if (angle < 0) {
    angle += 2 * CV_PI;
  }
	// Convert to degrees instead of radians
	angle = 180 * angle / CV_PI;
	// Calculate the size of the major and minor axes
	double halfMajorAxisSize = chiSquareVal * sqrt(eigenvalues.at<double>(0));
	double halfMinorAxisSize = chiSquareVal * sqrt(eigenvalues.at<double>(1));
	return cv::RotatedRect(mean, cv::Size2f(halfMajorAxisSize, halfMinorAxisSize), -1.0 * angle);
}
