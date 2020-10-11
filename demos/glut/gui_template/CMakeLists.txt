add_executable(gui_template src/application.cpp)

target_link_libraries(gui_template
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS gui_template
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
