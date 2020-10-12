add_executable(finite_element_method src/application.cpp)

target_link_libraries(finite_element_method
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    TinyXML
    OpenTissue
)

target_include_directories(finite_element_method
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

install(
  TARGETS finite_element_method
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
