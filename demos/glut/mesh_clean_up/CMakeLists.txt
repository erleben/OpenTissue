add_executable(mesh_clean_up src/application.cpp)

target_link_libraries(mesh_clean_up
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
)

install(
  TARGETS mesh_clean_up
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
