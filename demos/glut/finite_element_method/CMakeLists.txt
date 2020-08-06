add_executable(finite_element_method src/application.cpp)

target_link_libraries(finite_element_method
  PRIVATE
    OpenGL::OpenGL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    ${IL_LIBRARIES}
    ${ILU_LIBRARIES}
    ${ILUT_LIBRARIES}
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
