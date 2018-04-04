add_executable(srrg_semantic_mapper_app srrg_semantic_mapper_app.cpp)

target_link_libraries(srrg_semantic_mapper_app
  srrg_semantic_mapper_library
  srrg_semantic_mapper_viewers_library
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
