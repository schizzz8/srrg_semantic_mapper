#include "semantic_map.h"

namespace YAML {
  using namespace srrg_semantic_mapper;
  template<>
  struct convert<Object> {
    static Node encode(const Object &obj) {
      Node node;
      node["id"] = obj.id();
      node["type"] = obj.type();
      return node;
    }

    static bool decode(const Node& node, Object &obj) {
      if(!node.IsMap() || node.size() != 2) {
        return false;
      }

      obj.id() = node["id"].as<int>();
      obj.type() = node["type"].as<std::string>();
      return true;
    }
  };
}

namespace srrg_semantic_mapper {

  SemanticMap::SemanticMap(){}

  void SemanticMap::serialize(const std::string &filename){
    YAML::Node node;

    for(size_t i=0; i < _objects.size(); ++i){
      char buffer[80];
      sprintf(buffer,"object_%lu",i);

      node[buffer] = *_objects[i];
    }

    std::ofstream fout(filename);
    fout << node;
  }

  void SemanticMap::deserialize(const std::string &filename){
    std::ifstream fin(filename);
    YAML::Node node = YAML::Load(fin);

    _objects.resize(node.size());
    for(size_t i=0; i <node.size(); ++i){
      char buffer[80];
      sprintf(buffer,"object_%lu",i);

      Object obj = node[buffer].as<Object>();
      ObjectPtr obj_ptr = ObjectPtr(new Object(obj));
      _objects[i] = obj_ptr;
    }
  }

}
