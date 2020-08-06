add_executable(vertex_mean_curvature src/application.cpp)

target_link_libraries(vertex_mean_curvature
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
  TARGETS vertex_mean_curvature
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )

