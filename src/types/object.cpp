#include "object.h"

namespace srrg_semantic_mapper{

  using namespace std;

  Object::Object(int id_,
                 std::string type_,
                 Eigen::Isometry3f pose_,
                 Eigen::Vector3f lower_,
                 Eigen::Vector3f upper_):
    _id(id_),
    _type(type_),
    _pose(pose_),
    _lower(lower_),
    _upper(upper_){

  }

  bool Object::operator <(const Object &o){
    return (this->_id < o.id());
  }

  bool Object::operator ==(const Object &o){
    return (this->_id == o.id());
  }

  void Object::merge(const ObjectPtr & o){
    if(o->lower().x() < _lower.x())
      _lower.x() = o->lower().x();
    if(o->upper().x() > _upper.x())
      _upper.x() = o->upper().x();
    if(o->lower().y() < _lower.y())
      _lower.y() = o->lower().y();
    if(o->upper().y() > _upper.y())
      _upper.y() = o->upper().y();
    if(o->lower().z() < _lower.z())
      _lower.z() = o->lower().z();
    if(o->upper().z() > _upper.z())
      _upper.z() = o->upper().z();

    _pose.translation() = (_lower+_upper)/2.0f;

  }
}

