add_executable(unit_outside_triangle src/unit_outside_tri.cpp)

target_link_libraries(unit_outside_triangle
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_outside_triangle
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_outside_triangle)

