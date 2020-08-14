add_executable(particle_system src/application.cpp)

target_link_libraries(particle_system
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    ${IL_LIBRARIES}
    ${ILU_LIBRARIES}
    ${ILUT_LIBRARIES}
    OpenTissue
)

install(
  TARGETS particle_system
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )

