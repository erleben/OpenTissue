add_executable(volume_fitting src/application.cpp)

target_link_libraries(volume_fitting
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    Qhull::libqhull
    OpenTissue
)

install(
  TARGETS volume_fitting
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
