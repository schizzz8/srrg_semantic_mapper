#pragma once
#include <string>
#include <vector>
#include <memory>

#include <srrg_types/cloud_3d.h>

#include <Eigen/Geometry>


namespace srrg_semantic_mapper {

  class Object;
  typedef std::shared_ptr<Object> ObjectPtr;
  typedef std::vector<ObjectPtr> ObjectVector;
  typedef std::pair<ObjectPtr,ObjectPtr> Association;

  class Object{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Object(int id_=-1,
           const std::string &type_="",
           const Eigen::Isometry3f &pose_=Eigen::Isometry3f::Identity(),
           const Eigen::Vector3f &lower_=Eigen::Vector3f::Zero(),
           const Eigen::Vector3f &upper_=Eigen::Vector3f::Zero(),
           const srrg_core::Cloud3D &cloud_=srrg_core::Cloud3D());

    bool operator < (const Object &o);
    bool operator == (const Object &o);

    void merge(const ObjectPtr &o);

    void draw() const;

    inline const int id() const {return _id;}
    inline int& id() {return _id;}
    inline const std::string& type() const {return _type;}
    inline std::string& type() {return _type;}
    inline const Eigen::Isometry3f& pose() const {return _pose;}
    inline Eigen::Isometry3f& pose() {return _pose;}
    inline const Eigen::Vector3f& lower() const {return _lower;}
    inline Eigen::Vector3f& lower() {return _lower;}
    inline const Eigen::Vector3f& upper() const {return _upper;}
    inline Eigen::Vector3f& upper() {return _upper;}

  private:
    int _id;
    std::string _type;
    Eigen::Isometry3f _pose;
    Eigen::Vector3f _lower;
    Eigen::Vector3f _upper;
    srrg_core::Cloud3D _cloud;
  };

}
