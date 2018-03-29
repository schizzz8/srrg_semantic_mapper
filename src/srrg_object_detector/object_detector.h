#pragma once

#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <srrg_types/types.hpp>

#include <types/detection.h>

namespace srrg_semantic_mapper{

  class ObjectDetector{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct Model{
      Model(std::string type_ = "",
            Eigen::Isometry3f pose_ = Eigen::Isometry3f::Identity(),
            Eigen::Vector3f min_ = Eigen::Vector3f::Zero(),
            Eigen::Vector3f max_ = Eigen::Vector3f::Zero()):
        _type(type_),
        _pose(pose_),
        _min(min_),
        _max(max_){}
      std::string _type;
      Eigen::Isometry3f _pose;
      Eigen::Vector3f _min;
      Eigen::Vector3f _max;
    };

    typedef std::pair<Eigen::Vector3f,Eigen::Vector3f> BoundingBox3D;
    typedef std::vector<BoundingBox3D> BoundingBoxes3D;
    typedef std::vector<Model> ModelVector;

    ObjectDetector(){}

    inline void setTransform(const Eigen::Isometry3f &transform_){_transform = transform_;}
    inline void setModels(const ModelVector &models_){_models = models_; _bounding_boxes.resize(_models.size());}

    void compute(DetectionVector &detections,
                 const srrg_core::Float3Image &points_image = srrg_core::Float3Image());

    inline const srrg_core::RGBImage& labelImage() const {return _label_image;}
    inline const BoundingBoxes3D& boundingBoxes() const {return _bounding_boxes;}

  protected:

    Eigen::Isometry3f _transform;
    ModelVector _models;
    BoundingBoxes3D _bounding_boxes;

    srrg_core::RGBImage _label_image;

  private:
    void computeWorldBoundingBoxes(DetectionVector &detections);

    inline bool inRange(const Eigen::Vector3f &point, const BoundingBox3D &bounding_box){
      return (point.x() >= bounding_box.first.x()-0.01 && point.x() <= bounding_box.second.x()+0.01 &&
              point.y() >= bounding_box.first.y()-0.01 && point.y() <= bounding_box.second.y()+0.01 &&
              point.z() >= bounding_box.first.z()-0.01 && point.z() <= bounding_box.second.z()+0.01);
    }

    void computeImageBoundingBoxes(DetectionVector &detections,
                                   const srrg_core::Float3Image &points_image = srrg_core::Float3Image());

    cv::Vec3b type2color(std::string type);

    void computeLabelImage(const DetectionVector &detections);

  };

}

