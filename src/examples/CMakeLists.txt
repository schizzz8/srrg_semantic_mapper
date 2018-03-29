add_library(srrg_semantic_mapper_library SHARED
  object.cpp object.h
  semantic_map.cpp semantic_map.h
  #semantic_mapper.cpp semantic_mapper.h
)

target_link_libraries(srrg_semantic_mapper_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
