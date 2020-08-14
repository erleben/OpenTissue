add_executable(character_animation src/application.cpp)

target_link_libraries(character_animation
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    ${IL_LIBRARIES}
    ${ILU_LIBRARIES}
    ${ILUT_LIBRARIES}
    TinyXML
    OpenTissue
)

target_include_directories(character_animation
    PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/bundled/
)

install(
  TARGETS character_animation
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
