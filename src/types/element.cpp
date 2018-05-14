#include "element.h"

namespace srrg_semantic_mapper {

  using namespace srrg_core;

  Element::Element(const std::__cxx11::string &type_,
                   const Eigen::Isometry3f &pose_,
                   const Eigen::Vector3f &min_,
                   const Eigen::Vector3f &max_):
    _type(type_),
    _pose(pose_),
    _min(min_),
    _max(max_){

    _camera_min << std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max();

    _camera_max << -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max();

    _model_min << std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max();

    _model_max << -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max();

  }

  void Element::setCameraMinMax(const Eigen::Isometry3f &transform){
    std::vector<Eigen::Vector3f> points(2);
    points[0] = transform*_pose*_min;
    points[1] = transform*_pose*_max;

    for(int i=0; i < 2; ++i){
      if(points[i].x()<_camera_min.x())
        _camera_min.x() = points[i].x();
      if(points[i].x()>_camera_max.x())
        _camera_max.x() = points[i].x();
      if(points[i].y()<_camera_min.y())
        _camera_min.y() = points[i].y();
      if(points[i].y()>_camera_max.y())
        _camera_max.y() = points[i].y();
      if(points[i].z()<_camera_min.z())
        _camera_min.z() = points[i].z();
      if(points[i].z()>_camera_max.z())
        _camera_max.z() = points[i].z();
    }
  }

  bool Element::inRange(const Eigen::Vector3f &point) const{
    return (point.x() >= _camera_min.x()-0.01 && point.x() <= _camera_max.x()+0.01 &&
            point.y() >= _camera_min.y()-0.01 && point.y() <= _camera_max.y()+0.01 &&
            point.z() >= _camera_min.z()-0.01 && point.z() <= _camera_max.z()+0.01);
  }

  void Element::addPoint(const Eigen::Vector3f &point){
    _cloud.push_back(RichPoint3D (point, Eigen::Vector3f::Zero(),1.0f));
  }

  void Element::computeModelBoundingBox(){
    for(size_t i=0; i<_cloud.size(); ++i){
      const Eigen::Vector3f &point = _cloud[i].point();

      if(point.x()<_model_min.x())
        _model_min.x() = point.x();
      if(point.x()>_model_max.x())
        _model_max.x() = point.x();
      if(point.y()<_model_min.y())
        _model_min.y() = point.y();
      if(point.y()>_model_max.y())
        _model_max.y() = point.y();
      if(point.z()<_model_min.z())
        _model_min.z() = point.z();
      if(point.z()>_model_max.z())
        _model_max.z() = point.z();
    }
  }

  void Element::print() const {
    std::cerr << ">>Type: " << _type << std::endl;
    std::cerr << ">>GT min: " << _min.transpose() << std::endl;
    std::cerr << ">>GT max: " << _max.transpose() << std::endl;
    std::cerr << ">>MY min: " << _model_min.transpose() << std::endl;
    std::cerr << ">>MY max: " << _model_max.transpose() << std::endl;
    std::cerr << std::endl;
  }

}
