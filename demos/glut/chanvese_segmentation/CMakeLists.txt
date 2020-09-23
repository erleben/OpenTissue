add_executable(chanvese_segmentation src/application.cpp)

target_link_libraries(chanvese_segmentation
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
    Boost::headers
    Graphics
)

target_compile_definitions(chanvese_segmentation 
  PRIVATE
   OT_INJECT_MAIN
)

install(
  TARGETS chanvese_segmentation
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
