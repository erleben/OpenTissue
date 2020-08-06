add_executable(unit_compute_signed_distance_to_triangle src/unit_compute_signed_distance_to_triangle.cpp)

target_link_libraries(unit_compute_signed_distance_to_triangle
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_compute_signed_distance_to_triangle
  RUNTIME DESTINATION  bin/units
  )


ot_add_test(unit_compute_signed_distance_to_triangle)
