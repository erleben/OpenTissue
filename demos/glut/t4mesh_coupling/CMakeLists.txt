add_executable(t4mesh_coupling src/application.cpp)

target_link_libraries(t4mesh_coupling
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
  TARGETS t4mesh_coupling
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )


