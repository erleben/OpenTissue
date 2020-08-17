#-----------------------------------------------------------------------------
#
# Create OpenTissue exportable target
#

if(NOT TARGET OpenTissue)
  add_library(OpenTissue INTERFACE)
endif()

target_link_libraries(OpenTissue
  INTERFACE
    Boost::disable_autolinking
)

target_include_directories(OpenTissue
  INTERFACE
    $<INSTALL_INTERFACE:include/OpenTissue>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
)

target_compile_definitions(OpenTissue
  INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:_SCL_SECURE_NO_WARNINGS>
    $<$<CXX_COMPILER_ID:MSVC>:_SCL_SECURE_NO_DEPRECATE>
    $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>
    $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_DEPRECATE>
    $<$<CXX_COMPILER_ID:MSVC>:_SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS>
    $<$<CXX_COMPILER_ID:GNU>:_USE_MATH_DEFINES>
    $<$<CXX_COMPILER_ID:GNU>:GL_SILENCE_DEPRECATION>
)

#-------------------------------------------------------------------------------
#
# Generate configuration.h file
# This way platform and system specific settings can be added to the
# configuration.h file
#
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/configuration.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/configuration.h"
  @ONLY
)
