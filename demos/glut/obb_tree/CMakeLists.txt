add_executable(obb_tree src/application.cpp)

target_link_libraries(obb_tree
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
  TARGETS obb_tree
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
