add_executable(shallow_water_equations src/application.cpp)

target_link_libraries(shallow_water_equations
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS shallow_water_equations
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
