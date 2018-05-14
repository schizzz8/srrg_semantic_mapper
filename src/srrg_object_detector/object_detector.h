#pragma once

#include <types/element.h>

#include <srrg_types/types.hpp>

namespace srrg_semantic_mapper{

  class ObjectDetector{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      inline void setTransform(const Eigen::Isometry3f &transform_){_transform = transform_;}
      inline void setElements(const ElementPtrVector &elements_){_elements = elements_;}

      void compute(const srrg_core::Float3Image &points_image = srrg_core::Float3Image());

      inline const ElementPtrVector &elements() const {return _elements;}

    protected:

      Eigen::Isometry3f _transform;
      ElementPtrVector _elements;
  };
}
