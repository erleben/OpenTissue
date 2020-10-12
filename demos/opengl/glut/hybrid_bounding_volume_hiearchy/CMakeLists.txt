add_executable(hybrid_bounding_volume_hierarchy src/application.cpp)

target_link_libraries(hybrid_bounding_volume_hierarchy
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
  TARGETS hybrid_bounding_volume_hierarchy
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
