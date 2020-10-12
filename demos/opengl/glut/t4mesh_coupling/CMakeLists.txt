add_executable(t4mesh_coupling src/application.cpp)

target_link_libraries(t4mesh_coupling
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS t4mesh_coupling
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
