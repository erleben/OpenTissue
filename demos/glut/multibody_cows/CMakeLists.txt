add_executable(multibody_cows src/application.cpp)

target_link_libraries(multibody_cows
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
  TARGETS multibody_cows
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )

