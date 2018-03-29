#include "semantic_map.h"

using namespace srrg_boss;

SemanticMap::SemanticMap(){}

SemanticMap::~SemanticMap(){
  if (_objects.size()) {
    clear();
  }
}

void SemanticMap::clear(){
  for(int i=0; i<_objects.size(); ++i)
    delete _objects[i];

  _objects.clear();
}

void SemanticMap::addObject(Object *object_){
  _objects.push_back(object_);
}

void SemanticMap::serialize(srrg_boss::ObjectData &data, srrg_boss::IdContext &context){
  ArrayData* map_array = new ArrayData;
  for(int i=0; i<_objects.size(); ++i){
    ObjectData* odata=new ObjectData;
    _objects[i]->serialize(*odata,context);
    map_array->add(odata);
  }
  data.setField("semantic_map",map_array);
}

void SemanticMap::deserialize(ObjectData &data, IdContext &context){
  clear();

  ArrayData& odata_vector = data.getField("semantic_map")->getArray();
  for(int i=0; i<odata_vector.size(); ++i){
    ObjectData& odata=odata_vector[i].getObject();
    Object* obj=new Object;
    obj->deserialize(odata,context);
    addObject(obj);
  }
}

BOSS_REGISTER_CLASS(SemanticMap);
