#pragma once

#include "object.h"

#include <fstream>
#include <yaml-cpp/yaml.h>

namespace srrg_semantic_mapper {
  class SemanticMap {
  public:
    SemanticMap();

    void addObject(const ObjectPtr& object_){_objects.push_back(object_);}
    void clear(){_objects.clear();}
    size_t size() const {return _objects.size();}
    const ObjectPtr& operator[](size_t i) const {assert(i < _objects.size() && "Bound error!"); return _objects[i];}
    ObjectPtr& operator[](size_t i){assert(i < _objects.size() && "Bound error!"); return _objects[i];}

    inline const ObjectVector& objects() const {return _objects;}

    void serialize(const std::string &filename);
    void deserialize(const std::string &filename);

  private:
    ObjectVector _objects;
  };

}
