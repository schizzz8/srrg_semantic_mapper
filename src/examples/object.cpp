#include "object.h"

Object::Object(int id_,
               std::string type_,
               Eigen::Isometry3f pose_,
               Eigen::Vector3f min_,
               Eigen::Vector3f max_):
  _id(id_),
  _type(type_),
  _pose(pose_),
  _min(min_),
  _max(_max){

}

void Object::serialize(srrg_boss::ObjectData &data, srrg_boss::IdContext &context){
  Serializable::serialize(data,context);

  data.setInt("id",_id);
  srrg_core::t2v(_pose).toBOSS(data,"pose");
  _min.toBOSS(data,"min");
  _max.toBOSS(data,"_max");
}

void Object::deserialize(srrg_boss::ObjectData &data, srrg_boss::IdContext &context){
  Serializable::deserialize(data,context);

  _id = data.getInt("id");
  srrg_core::Vector6f v;
  v.fromBOSS(data,"pose");
  _pose=srrg_core::v2t(v);
  _min.fromBOSS(data,"min");
  _max.fromBOSS(data,"_max");
}
