#ifndef OBJECT_DEFINITION_H
#define OBJECT_DEFINITION_H

#include <machine_vision/color_filter.h>

#include <vector>
#include <string>

class ObjectType {
public:
  ObjectType(std::string name, std::vector<ColorFilter*> colorFilters);

private:
  ObjectType();

  std::string mName;
  std::vector<ColorFilter*> mColorFilters;
};

#endif
