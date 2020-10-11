add_executable(ellipsoid_decomposition src/application.cpp)

target_link_libraries(ellipsoid_decomposition
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
  TARGETS ellipsoid_decomposition
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
