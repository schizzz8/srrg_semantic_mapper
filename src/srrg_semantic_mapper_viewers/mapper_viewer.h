#pragma once

#include <qapplication.h>
#include <qglviewer.h>

#include <srrg_types/types.hpp>

#include <srrg_semantic_mapper/semantic_mapper.h>


#if QT_VERSION >= 0x050000
typedef qreal qglviewer_real;
#else
typedef float qglviewer_real;
#endif


namespace srrg_semantic_mapper{

  class StandardCamera: public qglviewer::Camera {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  protected:

    virtual qglviewer_real zFar() const { return 100; }
    virtual qglviewer_real zNear() const { return 0.1; }
  };

  class MapperViewer : public QGLViewer {
  public:
    MapperViewer();

    void init();

    virtual void draw();

    void setMapper(SemanticMapper *mapper_);

  protected:
    SemanticMapper *_mapper;

    SemanticMap *_global_map;
  };

}
