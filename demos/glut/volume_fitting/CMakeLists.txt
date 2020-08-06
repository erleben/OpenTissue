add_executable(volume_fitting src/application.cpp)

target_link_libraries(volume_fitting
  PRIVATE
    OpenGL::OpenGL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    ${IL_LIBRARIES}
    ${ILU_LIBRARIES}
    ${ILUT_LIBRARIES}
    Qhull::libqhull
    OpenTissue
)

install(
  TARGETS volume_fitting
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )


