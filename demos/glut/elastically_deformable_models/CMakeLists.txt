add_executable(elastically_deformable_models src/application.cpp)

target_link_libraries(elastically_deformable_models
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
)

install(
  TARGETS elastically_deformable_models
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
