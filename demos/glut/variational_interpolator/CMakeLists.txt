add_executable(variational_interpolator src/application.cpp)

target_link_libraries(variational_interpolator
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
)
install(
  TARGETS variational_interpolator
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
