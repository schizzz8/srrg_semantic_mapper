add_library(srrg_semantic_mapper_library SHARED
  semantic_mapper.h semantic_mapper.cpp
)
target_link_libraries(srrg_semantic_mapper_library
  srrg_object_detector_library
  srrg_types_library
  ${catkin_LIBRARIES}
)

