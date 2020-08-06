add_executable(hybrid_bounding_volume_hierarchy src/application.cpp)

target_link_libraries(hybrid_bounding_volume_hierarchy
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
  TARGETS hybrid_bounding_volume_hierarchy
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
