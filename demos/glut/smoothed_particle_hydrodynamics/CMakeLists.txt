add_executable(smoothed_particle_hydrodynamics src/application.cpp)

target_link_libraries(smoothed_particle_hydrodynamics
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
  TARGETS smoothed_particle_hydrodynamics
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
