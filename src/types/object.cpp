#include "object.h"
#include "srrg_gl_helpers/opengl_primitives.h"

namespace srrg_semantic_mapper{

  using namespace std;
  using namespace srrg_gl_helpers;

  Object::Object(int id_,
                 const string & type_,
                 const Eigen::Isometry3f & pose_,
                 const Eigen::Vector3f & lower_,
                 const Eigen::Vector3f & upper_,
                 const srrg_core::Cloud3D & cloud_):
    _id(id_),
    _type(type_),
    _pose(pose_),
    _lower(lower_),
    _upper(upper_),
    _cloud(cloud_) {}

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

  void Object::draw() const {

    _cloud.draw();

    const Eigen::Vector3f size = (_upper - _lower);

    const Eigen::Vector3f centroid = (_lower + _upper)/2.0f;
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.translation() = centroid;

    glPushMatrix();
    glMultMatrix(transform);
//    glMultMatrix(_pose);
    glColor4f(1.0,0.0,0.0,1.0);
    drawBoxWireframe(size.x(),size.y(),size.z());
    glPopMatrix();

  }
}

