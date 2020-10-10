add_executable(mesh_top_down_bvh src/application.cpp)

target_link_libraries(mesh_top_down_bvh
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS mesh_top_down_bvh
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
