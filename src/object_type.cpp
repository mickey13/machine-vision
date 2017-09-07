#include <machine_vision/object_type.h>

ObjectType::ObjectType(std::string name, std::vector<ColorFilter*> colorFilters) {
  this->mName = name;
  this->mColorFilters = colorFilters;
}
