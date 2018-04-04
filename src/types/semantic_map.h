#pragma once

#include "object.h"

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace srrg_semantic_mapper {
  class SemanticMap {
  public:
    inline void addObject(const ObjectPtr& object_){_objects.push_back(object_);}
    inline void clear(){_objects.clear();}
    inline size_t size() const {return _objects.size();}
    inline const ObjectPtr& operator[](size_t i) const {assert(i < _objects.size() && "Bound error!"); return _objects[i];}
    inline ObjectPtr& operator[](size_t i){assert(i < _objects.size() && "Bound error!"); return _objects[i];}

    inline const ObjectVector& objects() const {return _objects;}

    void draw() const;

    void serialize(const std::string &filename);
    void deserialize(const std::string &filename);

  private:
    ObjectVector _objects;
  };

}
