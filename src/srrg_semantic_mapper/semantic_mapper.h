#pragma once

#include <iostream>

#include <types/semantic_map.h>
#include <types/detection.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <srrg_types/types.hpp>
#include <srrg_image_utils/depth_utils.h>
#include <srrg_image_utils/point_image_utils.h>

#include <srrg_object_detector/object_detector.h>

namespace srrg_semantic_mapper {

class SemanticMapper{
public:
    SemanticMapper();

    ~SemanticMapper();

    inline void setK(const Eigen::Matrix3f& K_){_K = K_; _invK = _K.inverse();}

    void setImages(const srrg_core::RGBImage& rgb_image_,
                   const srrg_core::RawDepthImage& raw_depth_image_);

    void extractObjects();
    void findAssociations();
    void mergeMaps();

    const ObjectDetector &detector() const {return _detector;}
    ObjectDetector &detector() {return _detector;}

    inline SemanticMap *globalMap() {return _global_map;}
    inline const Eigen::Isometry3f &globalT() const {return _globalT;}


protected:
    float _raw_depth_scale;
    float _min_distance, _max_distance;

    Eigen::Matrix3f _K,_invK;

    cv::Mat _rgb_image;
    cv::Mat _depth_image;

    int _rows;
    int _cols;
    srrg_core::Float3Image _points_image;

    ObjectDetector _detector;
    DetectionVector _detections;

    bool _local_set;
    bool _global_set;

    SemanticMap *_local_map;
    SemanticMap *_global_map;
    Eigen::Isometry3f _globalT;

    std::vector<Association> _associations;

private:
    srrg_core::Vector3fVector unproject(const std::vector<Eigen::Vector2i> &pixels);
    void getLowerUpper3d(const srrg_core::Vector3fVector &points, Eigen::Vector3f &lower, Eigen::Vector3f &upper);
    int associationID(const ObjectPtr &local);
};

}
