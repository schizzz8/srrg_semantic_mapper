add_library(srrg_object_detector_library SHARED
  object_detector.cpp object_detector.h
)

target_link_libraries(srrg_object_detector_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
