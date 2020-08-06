add_executable(unit_grid src/unit_grid.cpp)

target_link_libraries(unit_grid
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_grid
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_grid)
