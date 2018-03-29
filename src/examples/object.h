#pragma once

#include <string>

//#include <Eigen/Core>
//#include <Eigen/Geometry>

#include <srrg_types/defs.h>

class Object;
typedef std::vector<Object*> Objects;

class Object : srrg_boss::Serializable {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Object(int id_=-1,
         std::string type_="",
         Eigen::Isometry3f pose_=Eigen::Isometry3f::Identity(),
         Eigen::Vector3f min_=Eigen::Vector3f::Zero(),
         Eigen::Vector3f max_=Eigen::Vector3f::Zero());

  virtual void serialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) override;
  virtual void deserialize(srrg_boss::ObjectData& data, srrg_boss::IdContext& context) override;

private:
  int _id;
  std::string _type;
  Eigen::Isometry3f _pose;
  Eigen::Vector3f _min;
  Eigen::Vector3f _max;
};
