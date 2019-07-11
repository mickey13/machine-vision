#ifndef ROI_MARKER_H
#define ROI_MARKER_H

#include <machine_vision/color_filter.h>

class RoiMarker {
public:
  RoiMarker(const ColorFilter& colorFilter);

private:
  RoiMarker();

  ColorFilter mColorFilter;
};

#endif
