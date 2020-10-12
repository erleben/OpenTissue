add_executable(smoothed_particle_hydrodynamics src/application.cpp)

target_link_libraries(smoothed_particle_hydrodynamics
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS smoothed_particle_hydrodynamics
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
