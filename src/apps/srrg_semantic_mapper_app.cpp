#include <iostream>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <srrg_semantic_mapper/semantic_mapper.h>

using namespace srrg_semantic_mapper;

void deserializeTransform(const char* filename, Eigen::Isometry3f &transform);
void deserializeModels(const char* filename, ObjectDetector::ModelVector &models);

int main(int argc, char** argv){

  //K
  Eigen::Matrix3f K;
  K << 554.25,    0.0, 320.5,
          0.0, 554.25, 240.5,
          0.0,    0.0,   1.0;

  SemanticMapper mapper;
  mapper.setK(K);

  std::string line;
  std::ifstream data(argv[1]);

  if(data.is_open()){
    while(std::getline(data,line)){
      std::istringstream iss(line);
      int seq;
      std::string rgb_filename,depth_filename,rgbd_filename,logical_filename,models_filename;
      iss>>seq>>rgb_filename>>depth_filename>>rgbd_filename>>logical_filename>>models_filename;

      //read rgbd transform
      Eigen::Isometry3f rgbd_transform = Eigen::Isometry3f::Identity();
      deserializeTransform(rgbd_filename.c_str(),rgbd_transform);

      //read logical transform
      Eigen::Isometry3f logical_transform = Eigen::Isometry3f::Identity();
      deserializeTransform(logical_filename.c_str(),logical_transform);

      //read models
      ObjectDetector::ModelVector models;
      deserializeModels(models_filename.c_str(),models);

      //read images
      cv::Mat rgb_image = cv::imread(rgb_filename,cv::IMREAD_UNCHANGED);
      cv::Mat raw_depth_image = cv::imread(depth_filename,cv::IMREAD_UNCHANGED);

      mapper.detector().setTransform(rgbd_transform.inverse()*logical_transform);
      mapper.detector().setModels(models);
      mapper.setImages(rgb_image,raw_depth_image);

//      cv::Mat label_image = mapper.detector().labelImage().clone();
//      cv::imshow("label_image",label_image);
//      cv::waitKey();


    }
  }

  data.close();

  return 0;
}

void deserializeTransform(const char * filename, Eigen::Isometry3f &transform){
  std::ifstream fin(filename);
  std::string line;

  if(fin.is_open()){
    if(std::getline(fin,line)){
      std::istringstream iss(line);
      double px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22;
      iss >>px>>py>>pz>>r00>>r01>>r02>>r10>>r11>>r12>>r20>>r21>>r22;
      transform.translation()=Eigen::Vector3f(px,py,pz);
      Eigen::Matrix3f R;
      R << r00,r01,r02,r10,r11,r12,r20,r21,r22;
      transform.linear().matrix() = R;
    }
  }

  fin.close();
}

void deserializeModels(const char * filename, ObjectDetector::ModelVector &models){
  std::ifstream fin(filename);
  std::string line;

  if(fin.is_open()){
    while(std::getline(fin,line)){
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
  }

  fin.close();
}
