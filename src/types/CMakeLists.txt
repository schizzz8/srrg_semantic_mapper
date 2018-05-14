add_library(srrg_types_library SHARED
  object.cpp object.h
  element.cpp element.h
  semantic_map.cpp semantic_map.h
)

target_link_libraries(srrg_types_library
  yaml-cpp
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
