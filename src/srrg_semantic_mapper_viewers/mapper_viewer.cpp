#include "mapper_viewer.h"
#include "srrg_gl_helpers/opengl_primitives.h"

namespace srrg_semantic_mapper{

  using namespace srrg_gl_helpers;

  MapperViewer::MapperViewer(){
    StandardCamera * cam =new StandardCamera();
    setCamera(cam);

    _mapper = nullptr;
  }

  void MapperViewer::init(){
    QColor white = QColor(Qt::white);
    setBackgroundColor(white);
  }

  void MapperViewer::setMapper(SemanticMapper *mapper_){
    _mapper = mapper_;
    _global_map = _mapper->globalMap();
  }

  void MapperViewer::draw() {
    _global_map = _mapper->globalMap();

    //draw Map
    glPushMatrix();
    glMultMatrix(_mapper->globalT().inverse());
    _global_map->draw();
    glPopMatrix();
  }
}
