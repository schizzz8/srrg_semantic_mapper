#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <srrg_types/cloud_3d.h>

namespace srrg_semantic_mapper{

  class Element{

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      Element(const std::string &type_ = "",
              const Eigen::Isometry3f &pose_ = Eigen::Isometry3f::Identity(),
              const Eigen::Vector3f &min_ = Eigen::Vector3f::Zero(),
              const Eigen::Vector3f &max_ = Eigen::Vector3f::Zero());

      void setCameraMinMax(const Eigen::Isometry3f &transform);

      bool inRange(const Eigen::Vector3f &point) const;

      void addPoint(const Eigen::Vector3f &point);

      void computeModelBoundingBox();

      void print() const;

      inline const std::string &type() const {return _type;}
      inline const Eigen::Isometry3f &pose() const {return _pose;}
      inline const Eigen::Vector3f &min() const {return _model_min;}
      inline const Eigen::Vector3f &max() const {return _model_max;}
      inline const srrg_core::Cloud3D &cloud() const {return _cloud;}

    protected:

      std::string _type;
      Eigen::Isometry3f _pose;

      Eigen::Vector3f _min;
      Eigen::Vector3f _max;

      Eigen::Vector3f _camera_min;
      Eigen::Vector3f _camera_max;

      Eigen::Vector3f _model_min;
      Eigen::Vector3f _model_max;

      srrg_core::Cloud3D _cloud;

  };

  typedef std::vector<Element*> ElementPtrVector;

}
