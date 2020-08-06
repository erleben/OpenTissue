add_executable(damped_wave_equations src/application.cpp)

target_link_libraries(damped_wave_equations
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
  TARGETS damped_wave_equations
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
