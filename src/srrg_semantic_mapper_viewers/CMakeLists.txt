add_library(srrg_semantic_mapper_viewers_library SHARED
  mapper_viewer.cpp mapper_viewer.h
)

target_link_libraries(srrg_semantic_mapper_viewers_library
  srrg_semantic_mapper_library
  ${catkin_LIBRARIES}
  ${QGLVIEWER_LIBRARY} 
  ${SRRG_QT_LIBRARIES} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
)
