include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_projection src/unit_projection.cpp)

target_link_libraries(unit_projection 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_projection
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_projection)

