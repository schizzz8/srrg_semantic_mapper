#include "object_detector.h"

namespace srrg_semantic_mapper{

  using namespace std;
  using namespace srrg_core;

  void ObjectDetector::compute(const Float3Image &points_image){

    assert(_elements.size() && "[ObjectDetector][compute]: Elements vector empty!");

    //init elements
    for(size_t i=0; i<_elements.size(); ++i)
      _elements[i]->setCameraMinMax(_transform);

    //populate elements cloud
    for(size_t r=0; r<points_image.rows; ++r){
      const cv::Vec3f* point_ptr=points_image.ptr<const cv::Vec3f>(r);
      for(size_t c=0; c<points_image.cols; ++c, ++point_ptr){

        const cv::Vec3f& p = *point_ptr;

        if(cv::norm(p) < 1e-3)
          continue;

        const Eigen::Vector3f point(p[0],p[1],p[2]);

        //check if point is in element range
        for(size_t i=0; i<_elements.size(); ++i)
          if(_elements[i]->inRange(point)){
            _elements[i]->addPoint(point);
            break;
          }
      }

      //compute bounding box
      for(size_t i=0; i<_elements.size(); ++i)
        _elements[i]->computeModelBoundingBox();
    }


  }
}
