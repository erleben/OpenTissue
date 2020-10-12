add_executable(signed_distance_field_collision_detection src/application.cpp)

target_link_libraries(signed_distance_field_collision_detection
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::glew
    PNG::PNG
    OpenTissue
)

install(
  TARGETS signed_distance_field_collision_detection
  RUNTIME DESTINATION  bin/demos/glut/
  COMPONENT Demos
  )
