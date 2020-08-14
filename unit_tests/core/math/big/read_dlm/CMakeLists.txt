include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_read_dlm src/unit_read_dlm.cpp)

target_link_libraries(unit_read_dlm
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

add_custom_command(TARGET unit_read_dlm POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${PROJECT_SOURCE_DIR}/demos/data/dlm"
    "${CMAKE_CURRENT_BINARY_DIR}/dlm"
    )

install(
  TARGETS unit_read_dlm
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_read_dlm)

