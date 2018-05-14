#pragma once

#include <iostream>
#include <fstream>
#include <memory>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace srrg_semantic_mapper{

  class Detection;
  typedef std::shared_ptr<Detection> DetectionPtr;
  typedef std::vector<DetectionPtr> DetectionVector;

  class Detection{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Detection(const std::string& type_="",
              const Eigen::Vector2i& top_left_ = Eigen::Vector2i(10000,10000),
              const Eigen::Vector2i& bottom_right_ = Eigen::Vector2i(-10000,-10000),
              const std::vector<Eigen::Vector2i>& pixels_ = std::vector<Eigen::Vector2i>(640*480));

    inline const std::string& type() const {return _type;}
    inline std::string& type() {return _type;}
    inline const Eigen::Vector2i& topLeft() const {return _top_left;}
    inline Eigen::Vector2i& topLeft() {return _top_left;}
    inline const Eigen::Vector2i& bottomRight() const {return _bottom_right;}
    inline Eigen::Vector2i& bottomRight() {return _bottom_right;}
    inline const std::vector<Eigen::Vector2i>& pixels() const {return _pixels;}
    inline std::vector<Eigen::Vector2i>& pixels() {return _pixels;}
    inline const int size() const {return _size;}
    inline int &size() {return _size;}

  private:
    std::string _type;
    Eigen::Vector2i _top_left;
    Eigen::Vector2i _bottom_right;
    std::vector<Eigen::Vector2i> _pixels;
    int _size;
  };

}

