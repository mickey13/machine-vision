#ifndef DISTORTION_CORRECTOR_H
#define DISTORTION_CORRECTOR_H

#include <machine_vision/ocam_functions.h>

#include <string>
#include <cv_bridge/cv_bridge.h>

class DistortionCorrector {
public:
  DistortionCorrector(std::string calibrationFile, int imageHeight, int imageWidth);
  cv::Mat undistortImage(cv::Mat imageFrame);

private:
  DistortionCorrector();
  void debug() const;

  struct ocam_model mOcamModel;
  int mImageHeight;
  int mImageWidth;
};

#endif
