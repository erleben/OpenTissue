add_executable(t4_cpu_scan src/application.cpp)

target_link_libraries(t4_cpu_scan
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    OpenTissue
)

install(
  TARGETS t4_cpu_scan
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
