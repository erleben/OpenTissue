add_executable(character_animation src/application.cpp)

target_link_libraries(character_animation
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLEW::glew
    PNG::PNG
    TinyXML
    OpenTissue
    OpenTissueGraphics
    glfw
)

target_compile_definitions(character_animation
  PRIVATE
   OT_INJECT_MAIN
)

install(
  TARGETS character_animation
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
