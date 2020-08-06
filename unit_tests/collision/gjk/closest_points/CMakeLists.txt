add_executable(unit_closest_points src/unit_closest_points.cpp)

target_link_libraries(unit_closest_points
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_closest_points
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_closest_points)

