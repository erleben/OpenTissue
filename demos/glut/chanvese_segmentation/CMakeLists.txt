add_executable(chanvese_segmentation src/application.cpp)

target_link_libraries(chanvese_segmentation
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS chanvese_segmentation
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
