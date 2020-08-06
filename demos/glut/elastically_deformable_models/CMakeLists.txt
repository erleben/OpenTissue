add_executable(elastically_deformable_models src/application.cpp)

target_link_libraries(elastically_deformable_models
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
  TARGETS elastically_deformable_models
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
