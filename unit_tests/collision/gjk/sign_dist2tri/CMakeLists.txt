add_executable(unit_signed_distance_to_triangle src/unit_sign_dist2tri.cpp)

target_link_libraries(unit_signed_distance_to_triangle
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_signed_distance_to_triangle
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_signed_distance_to_triangle)

