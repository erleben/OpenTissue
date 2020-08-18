include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_const_bounds src/unit_const_bounds.cpp)

target_link_libraries(unit_const_bounds 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_const_bounds
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_const_bounds)

