#pragma once

#include "object.h"

class SemanticMap : public srrg_boss::Serializable {
public:
  SemanticMap();
  virtual ~SemanticMap();

  void addObject(Object* object_);

  void clear();

  virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) override;
  virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) override;

private:
  Objects _objects;
};
