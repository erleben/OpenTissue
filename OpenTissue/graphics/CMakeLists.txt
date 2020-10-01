
if(OPENTISSUE_ENABLE_GLUT)
  set(GRAPHICS_GLUT_SRCS 
    glut/glut_window.cpp 
    glut/glut_application.cpp
  )
endif()

add_library(OpenTissueGraphics 
  ${GRAPHICS_GLUT_SRCS}
)

target_link_libraries(OpenTissueGraphics 
  PRIVATE headers
)

target_compile_definitions(OpenTissueGraphics INTERFACE OT_USE_GLUT)

#-------------------------------------------------------------------------------
#
# Generate export headers
#
include(GenerateExportHeader)
generate_export_header(OpenTissueGraphics
  BASE_NAME graphics
  EXPORT_MACRO_NAME GRAPHICS_API
  DEPRECATED_MACRO_NAME GRAPHICS_DEPRECATED_API
  PREFIX_NAME OT_
  ${NO_BUILD_DEPRECATED}
)

