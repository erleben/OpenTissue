add_executable(damped_wave_equations src/application.cpp)

target_link_libraries(damped_wave_equations
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
)

install(
  TARGETS damped_wave_equations
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
