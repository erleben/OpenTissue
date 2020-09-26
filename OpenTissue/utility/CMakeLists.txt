
add_library(Graphics 
  glut/glut_window.cpp 
  glut/glut_application.cpp
)

target_link_libraries(Graphics 
  PRIVATE headers
)

#-------------------------------------------------------------------------------
#
# Generate export headers
#
include(GenerateExportHeader)
generate_export_header(Graphics
  EXPORT_MACRO_NAME GRAPHICS_API
  DEPRECATED_MACRO_NAME GRAPHICS_DEPRECATED_API
  PREFIX_NAME OT_
  ${NO_BUILD_DEPRECATED}
)

