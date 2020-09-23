
add_library(Graphics 
  glut/glut_window.cpp 
  glut/glut_application.cpp
)

target_link_libraries(Graphics 
  PRIVATE headers
)

