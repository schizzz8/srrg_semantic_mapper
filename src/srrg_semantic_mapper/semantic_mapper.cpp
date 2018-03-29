#include "semantic_mapper.h"

namespace srrg_semantic_mapper{

  using namespace std;
  using namespace srrg_core;

  SemanticMapper::SemanticMapper(){
    _globalT = Eigen::Isometry3f::Identity();

    _min_distance = 0.02;
    _max_distance = 5.0;

    _local_set = false;
    _global_set = false;
  }

  void SemanticMapper::readData(char *filename){

    std::string line;
    std::ifstream data(filename);

    Eigen::Isometry3f rgbd_camera_transform = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f logical_camera_transform = Eigen::Isometry3f::Identity();
    ObjectDetector::ModelVector models;

    if(data.is_open()) {
      if(std::getline(data,line)) {
        std::istringstream iss(line);
        double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
        iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
        rgbd_camera_transform.translation()=Eigen::Vector3f(px,py,pz);
        Eigen::Matrix3f R;
        R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
        rgbd_camera_transform.linear().matrix() = R;
      }
      if(std::getline(data,line)) {
        std::istringstream iss(line);
        double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
        iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
        logical_camera_transform.translation()=Eigen::Vector3f(px,py,pz);
        Eigen::Matrix3f R;
        R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
        logical_camera_transform.linear().matrix() = R;
      }
      int n;
      if(std::getline(data,line)) {
        std::istringstream iss(line);
        iss >> n;
      }
      for(int i=0; i<n; i++){
        std::getline(data,line);
        std::istringstream iss(line);
        std::string type;
        double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
        double minx,miny,minz,maxx,maxy,maxz;
        iss >> type;
        Eigen::Isometry3f model_pose=Eigen::Isometry3f::Identity();
        iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
        model_pose.translation()=Eigen::Vector3f(px,py,pz);
        Eigen::Matrix3f R;
        R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
        model_pose.linear().matrix() = R;
        iss >> minx>>miny>>minz>>maxx>>maxy>>maxz;
        Eigen::Vector3f min(minx,miny,minz);
        Eigen::Vector3f max(maxx,maxy,maxz);

        models.push_back(ObjectDetector::Model (type,model_pose,min,max));
      }
      data.close();
    }

    std::cerr << "RGBD camera pose" << std::endl;
    std::cerr << "position:" << std::endl;
    std::cerr << rgbd_camera_transform.translation().transpose() << std::endl;
    std::cerr << "orientation:" << std::endl;
    std::cerr << rgbd_camera_transform.linear().matrix() << std::endl << std::endl;

    std::cerr << "Logical camera pose" << std::endl;
    std::cerr << "position:" << std::endl;
    std::cerr << logical_camera_transform.translation().transpose() << std::endl;
    std::cerr << "orientation:" << std::endl;
    std::cerr << logical_camera_transform.linear().matrix() << std::endl << std::endl;

    _detector.setTransform(rgbd_camera_transform.inverse()*logical_camera_transform);

    int num_models=models.size();
    std::cerr << "Detected " << num_models << " models" << std::endl << std::endl;
    for(int i=0; i<num_models; ++i){
      const ObjectDetector::Model &model = models[i];
      std::cerr << model._type << std::endl;
      const Eigen::Isometry3f model_transform=model._pose;
      std::cerr << "position:" << std::endl;
      std::cerr << model_transform.translation().transpose() << std::endl;
      std::cerr << "orientation:" << std::endl;
      std::cerr << model_transform.linear().matrix() << std::endl;
      std::cerr << "Min:" << std::endl;
      std::cerr << model._min.transpose() << std::endl;
      std::cerr << "Max:" << std::endl;
      std::cerr << model._max.transpose() << std::endl << std::endl;
    }

    _detector.setModels(models);

  }
  void SemanticMapper::setImages(const srrg_core::RGBImage &rgb_image_,
                                 const srrg_core::RawDepthImage &raw_depth_image_){
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

    _detector.compute(_detections,_points_image);
  }

  Vector3fVector SemanticMapper::unproject(const std::vector<Eigen::Vector2i> &pixels){
    Vector3fVector points;
    for(int idx=0; idx < pixels.size(); ++idx){
      const Eigen::Vector2i& pixel = pixels[idx];
      int r = pixel.x();
      int c = pixel.y();
      const unsigned short& depth = _depth_image.at<const unsigned short>(r,c);
      float d = depth * _raw_depth_scale;

      if(d <= _min_distance)
        continue;

      if(d >= _max_distance)
        continue;

      Eigen::Vector3f camera_point = _invK * Eigen::Vector3f(c*d,r*d,d);
      Eigen::Vector3f map_point = _globalT*camera_point;
      //        std::cerr << map_point.transpose() << " ";
      points.push_back(map_point);

    }
    return points;
  }

  void SemanticMapper::getLowerUpper3d(const Vector3fVector &points, Eigen::Vector3f &lower, Eigen::Vector3f &upper){
    lower.x() = std::numeric_limits<float>::max();
    lower.y() = std::numeric_limits<float>::max();
    lower.z() = std::numeric_limits<float>::max();
    upper.x() = -std::numeric_limits<float>::max();
    upper.y() = -std::numeric_limits<float>::max();
    upper.z() = -std::numeric_limits<float>::max();

    for(int i=0; i < points.size(); ++i){

      if(points[i].x() < lower.x())
        lower.x() = points[i].x();
      if(points[i].x() > upper.x())
        upper.x() = points[i].x();
      if(points[i].y() < lower.y())
        lower.y() = points[i].y();
      if(points[i].y() > upper.y())
        upper.y() = points[i].y();
      if(points[i].z() < lower.z())
        lower.z() = points[i].z();
      if(points[i].z() > upper.z())
        upper.z() = points[i].z();
    }
  }

  void SemanticMapper::extractObjects(){

    bool populate_global = false;
    if(!_global_set){
      populate_global = true;
      _global_set = true;
    } else {
      _local_map.clear();
      _local_set = true;
    }

    for(int i=0; i < _detections.size(); ++i){

      const DetectionPtr& detection = _detections[i];

      cerr << detection->type() << ": [(";
      cerr << detection->topLeft().transpose() << ") - (" << detection->bottomRight().transpose() << ")]" << endl;

      if((detection->bottomRight()-detection->topLeft()).norm() < 1e-3)
        continue;

      std::cerr << "Unprojecting" << std::endl;
      Vector3fVector points = unproject(detection->pixels());

      Eigen::Vector3f lower,upper;
      getLowerUpper3d(points,lower,upper);

      Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
      pose.translation() = (upper+lower)/2.0f;

      std::string object_type = detection->type().substr(0,detection->type().find_first_of("_"));

      std::cerr << "BB: [(" << lower.transpose() << "," << upper.transpose() << ")]" << std::endl;

      ObjectPtr obj_ptr = ObjectPtr(new Object(i,
                                               object_type,
                                               pose,
                                               lower,
                                               upper));

      if(populate_global)
        _global_map.addObject(obj_ptr);
      else
        _local_map.addObject(obj_ptr);
    }
  }

  void SemanticMapper::findAssociations(){

    if(!_global_set || !_local_set)
      return;

    const int local_size = _local_map.size();
    const int global_size = _global_map.size();

    std::cerr << "[Data Association] ";
    std::cerr << "{Local Map size: " << local_size << "} ";
    std::cerr << "- {Global Map size: " << global_size << "}" << std::endl;

    _associations.clear();

    for(int i=0; i < global_size; ++i){
      const ObjectPtr &global = _global_map[i];
      const string &global_type = global->type();

      std::cerr << "\t>> Global: " << global_type;

      ObjectPtr local_best;
      float best_error = std::numeric_limits<float>::max();

      for(int j=0; j < local_size; ++j){
        const ObjectPtr &local = _local_map[j];
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
    for(int i=0; i < _local_map.size(); ++i){
      const ObjectPtr &local = _local_map[i];
      int association_id = associationID(local);
      if(association_id == -1){
        _global_map.addObject(local);
        added++;
        continue;
      } else {
        ObjectPtr &global_associated = _global_map[association_id];

        if(local->type() != global_associated->type())
          continue;

        global_associated->merge(local);
      }
    }
  }
}
