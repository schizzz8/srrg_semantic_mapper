add_library(srrg_types_library SHARED
  detection.cpp detection.h
  object.cpp object.h
  semantic_map.cpp semantic_map.h
)

target_link_libraries(srrg_types_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
