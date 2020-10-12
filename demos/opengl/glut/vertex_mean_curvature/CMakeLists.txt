add_executable(vertex_mean_curvature src/application.cpp)

target_link_libraries(vertex_mean_curvature
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS vertex_mean_curvature
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
