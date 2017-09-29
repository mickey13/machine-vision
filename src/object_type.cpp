#include <machine_vision/object_type.h>

ObjectType::ObjectType(std::string name, std::vector<ColorFilter*> colorFilters) {
  this->mName = name;
  this->mColorFilters = colorFilters;
}

cv::Mat ObjectType::filterImage(const cv::Mat& imageFrame) const {
  cv::Mat combinedFrame = cv::Mat::zeros(imageFrame.size(), CV_8UC1);
  for (std::vector<ColorFilter*>::const_iterator iter = this->mColorFilters.begin(); iter != this->mColorFilters.end(); ++iter) {
    cv::Mat thresholdFrame = (*iter)->filterImage(imageFrame);
    cv::bitwise_or(thresholdFrame, combinedFrame, combinedFrame);
  }
  return combinedFrame;
}

void ObjectType::debug() const {
  std::cout << "Object Type: " << this->mName << std::endl;
  for (std::vector<ColorFilter*>::const_iterator iter = this->mColorFilters.begin(); iter != this->mColorFilters.end(); ++iter) {
    (*iter)->test();
  }
}
