add_executable(t4mesh_tetgen_generator src/application.cpp)

target_link_libraries(t4mesh_tetgen_generator
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    DevIL::DevIL
    TetGen
    TinyXML
    OpenTissue
)

target_include_directories(t4mesh_tetgen_generator
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

install(
  TARGETS t4mesh_tetgen_generator
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
