add_executable(ellipsoid_decomposition src/application.cpp)

target_link_libraries(ellipsoid_decomposition
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
  TARGETS ellipsoid_decomposition
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )

