add_executable(variational_interpolator src/application.cpp)

target_link_libraries(variational_interpolator
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
  TARGETS variational_interpolator
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
