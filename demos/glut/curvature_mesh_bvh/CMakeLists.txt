add_executable(curvature_mesh_bvh src/application.cpp)

target_link_libraries(curvature_mesh_bvh
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
)

install(
  TARGETS curvature_mesh_bvh
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
