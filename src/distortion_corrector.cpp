#include <machine_vision/distortion_corrector.h>

#include <ros/ros.h>

DistortionCorrector::DistortionCorrector(std::string calibrationFile, int imageHeight, int imageWidth) {
  this->mImageHeight = imageHeight;
  this->mImageWidth = imageWidth;
  get_ocam_model(&(this->mOcamModel), &(calibrationFile[0]));
}

cv::Mat DistortionCorrector::undistortImage(cv::Mat imageFrame) {
  cv::Mat rectifiedDst, mapXP, mapYP;
  CvMat mapx, mapy;
  float sf = 4;

  try {
    mapXP = cv::Mat(this->mImageHeight, this->mImageWidth, CV_32FC1);
    mapYP = cv::Mat(this->mImageHeight, this->mImageWidth, CV_32FC1);

    mapx = mapXP;
    mapy = mapYP;

    create_perspecive_undistortion_LUT(&mapx, &mapy, &(this->mOcamModel), sf);

    cv::remap(imageFrame, rectifiedDst, mapXP, mapYP, cv::INTER_LINEAR);

    sensor_msgs::ImagePtr pubImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectifiedDst).toImageMsg();
    pubImage->header.stamp = ros::Time::now();
  }
  catch (std::exception exception) {
    ROS_ERROR("%s", exception.what());
  }

  return rectifiedDst;
}

void DistortionCorrector::debug() const {
  printf("pol =\n");
  for (int i = 0; i < this->mOcamModel.length_pol; ++i) {
    printf("\t%e\n", mOcamModel.pol[i]);
  }
  printf("invpol =\n");
  for (int i = 0; i < mOcamModel.length_invpol; ++i) {
    printf("\t%e\n", mOcamModel.invpol[i]);
  }
  printf("xc = %f\nyc = %f\n", mOcamModel.xc, mOcamModel.yc);
  printf("width = %d\nheight = %d\n", mOcamModel.width, mOcamModel.height);
}
