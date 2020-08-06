add_executable(obb_tree src/application.cpp)

target_link_libraries(obb_tree
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
  TARGETS obb_tree
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
