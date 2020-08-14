add_executable(versatile src/application.cpp)

target_link_libraries(versatile
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

target_include_directories(versatile
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

install(
  TARGETS versatile
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )



