add_executable(curvature_mesh_bvh src/application.cpp)

target_link_libraries(curvature_mesh_bvh
  PRIVATE
    OpenGL::OpenGL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    ${IL_LIBRARIES}
    ${ILU_LIBRARIES}
    ${ILUT_LIBRARIES}
    OpenTissue
)

install(
  TARGETS curvature_mesh_bvh
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )

