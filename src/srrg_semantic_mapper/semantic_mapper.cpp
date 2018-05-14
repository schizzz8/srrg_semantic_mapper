#include "semantic_mapper.h"

namespace srrg_semantic_mapper{

  using namespace std;
  using namespace srrg_core;

  SemanticMapper::SemanticMapper(){
    _globalT = Eigen::Isometry3f::Identity();

    _min_distance = 0.02;
    _max_distance = 5.0;
    _raw_depth_scale = 0.001;

    _local_set = false;
    _global_set = false;

    _local_map = new SemanticMap();
    _global_map = new SemanticMap();
  }

  SemanticMapper::~SemanticMapper(){
    delete _local_map;
    delete _global_map;
  }

  void SemanticMapper::setImages(const srrg_core::RGBImage &rgb_image_,
                                 const srrg_core::RawDepthImage &raw_depth_image_){

    bool populate_global = false;
    if(!_global_set){
      populate_global = true;
      _global_set = true;
    } else {
      _local_map->clear();
      _local_set = true;
    }

    //copy images
    _rgb_image = rgb_image_;

    _rows = _rgb_image.rows;
    _cols = _rgb_image.cols;

    //compute points image
    srrg_core::Float3Image directions_image;
    directions_image.create(_rows,_cols);
    initializePinholeDirections(directions_image,_K);
    _points_image.create(_rows,_cols);
    convert_16UC1_to_32FC1(_depth_image, raw_depth_image_);
    computePointsImage(_points_image,
                       directions_image,
                       _depth_image,
                       0.02f,
                       8.0f);

    //compute point cloud normals
    //    computeSimpleNormals(_normals_image,
    //                         _points_image,
    //                         3,
    //                         3,
    //                         8.0f);

    _detector.compute(_points_image);

    const ElementPtrVector &elements = _detector.elements();

    int k=0;
    for(int i=0; i < elements.size(); ++i){

      const Element* element = elements[i];

      if(element->cloud().empty())
        continue;

      element->print();

      ObjectPtr obj_ptr (new Object(k++,
                                    element->type(),
                                    element->pose(),
                                    element->min(),
                                    element->max(),
                                    element->cloud()));

      if(populate_global)
        _global_map->addObject(obj_ptr);
      else
        _local_map->addObject(obj_ptr);
    }

  }


  void SemanticMapper::findAssociations(){

    if(!_global_set || !_local_set)
      return;

    const int local_size = _local_map->size();
    const int global_size = _global_map->size();

    std::cerr << "[Data Association] ";
    std::cerr << "{Local Map size: " << local_size << "} ";
    std::cerr << "- {Global Map size: " << global_size << "}" << std::endl;

    _associations.clear();

    for(int i=0; i < global_size; ++i){
      const ObjectPtr &global = (*_global_map)[i];
      const string &global_type = global->type();

      std::cerr << "\t>> Global: " << global_type;

      ObjectPtr local_best;
      float best_error = std::numeric_limits<float>::max();

      for(int j=0; j < local_size; ++j){
        const ObjectPtr &local = (*_local_map)[j];
        const string &local_type = local->type();

        if(local_type != global_type)
          continue;

        Eigen::Vector3f e_c = local->pose().translation() - global->pose().translation();

        float error = e_c.transpose()*e_c;

        if(error<best_error){
          best_error = error;
          local_best = local;
        }
      }
      if(local_best->type() == "")
        continue;

      std::cerr << " - Local ID: " << local_best->type() << std::endl;

      _associations.push_back(Association(global,local_best));
    }
  }

  int SemanticMapper::associationID(const ObjectPtr &local){
    for(int i=0; i < _associations.size(); ++i)
      if(_associations[i].second->id() == local->id())
        return _associations[i].first->id();
    return -1;
  }

  void SemanticMapper::mergeMaps(){
    if(!_global_set || !_local_set)
      return;

    int added = 0, merged = 0;
    for(int i=0; i < _local_map->size(); ++i){
      const ObjectPtr &local = (*_local_map)[i];
      int association_id = associationID(local);
      if(association_id == -1){
        _global_map->addObject(local);
        added++;
        continue;
      } else {
        ObjectPtr &global_associated = (*_global_map)[association_id];

        if(local->type() != global_associated->type())
          continue;

        global_associated->merge(local);
        merged++;
      }
    }
  }

}


//  ObjectPtr SemanticMapper::objectFromDetection(const Detection &detection){

//    std::string object_type = detection.type().substr(0,detection.type().find_first_of("_"));
//    std::cerr << std::endl << object_type << ": ";

//    std::cerr << "[(" << detection.topLeft().transpose() << ") -";
//    std::cerr << " (" << detection.bottomRight().transpose() << ")]" << std::endl;

//    const std::vector<Eigen::Vector2i> &pixels = detection.pixels();
//    int num_pixels = pixels.size();
//    Cloud3D cloud;
//    cloud.resize(num_pixels);
//    int k=0;

//    Eigen::Vector3f min(std::numeric_limits<float>::max(),std::numeric_limits<float>::max(),std::numeric_limits<float>::max());
//    Eigen::Vector3f max(-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max(),-std::numeric_limits<float>::max());

//    std::cerr << "a";

//    for(int i=0; i<num_pixels; ++i){
//      const cv::Vec3f& cv_point = _points_image.at<const cv::Vec3f>(pixels[i].x(), pixels[i].y());
//      const cv::Vec3f& cv_normal = _normals_image.at<const cv::Vec3f>(pixels[i].x(), pixels[i].y());

//      if(cv::norm(cv_point) < 1e-3 || cv::norm(cv_normal) < 1e-3)
//        continue;

//      Eigen::Vector3f point(cv_point[0], cv_point[1],cv_point[2]);
//      Eigen::Vector3f normal(cv_normal[0], cv_normal[1],cv_normal[2]);
//      cloud[k] = RichPoint3D(point,normal,1.0f);
//      k++;

//      if(point.x() < min.x())
//        min.x() = point.x();
//      if(point.x() > max.x())
//        max.x() = point.x();
//      if(point.y() < min.y())
//        min.y() = point.y();
//      if(point.y() > max.y())
//        max.y() = point.y();
//      if(point.z() < min.z())
//        min.z() = point.z();
//      if(point.z() > max.z())
//        max.z() = point.z();
//    }

//    std::cerr << "b";

//    //check if object is empty
//    if(!k){
//      std::cerr << "empty object" << std::endl;
//      return nullptr;
//    }

//    cloud.resize(k);

//    std::cerr << "BB: [(" << min.transpose() << "," << max.transpose() << ")]" << std::endl;

//    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
//    pose.translation() = (max+min)/2.0f;

//    return ObjectPtr(new Object(0,object_type,pose,min,max,cloud));

//  }

//  void SemanticMapper::extractObjects(){

//    bool populate_global = false;
//    if(!_global_set){
//      populate_global = true;
//      _global_set = true;
//    } else {
//      _local_map->clear();
//      _local_set = true;
//    }

//    for(int i=0; i < _detections.size(); ++i){

//      const Detection& detection = *_detections[i];

//      if((detection.bottomRight()-detection.topLeft()).norm() < 1e-3 ||
//         (detection.bottomRight()-detection.topLeft()).norm() >= 2e+4)
//        continue;

//      ObjectPtr obj_ptr = objectFromDetection(detection);

//      if(!obj_ptr)
//        continue;

//      if(populate_global)
//        _global_map->addObject(obj_ptr);
//      else
//        _local_map->addObject(obj_ptr);
//    }
//  }

