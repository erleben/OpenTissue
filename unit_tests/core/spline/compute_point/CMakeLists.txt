add_executable(unit_compute_point_on_spline src/unit_compute_point.cpp)

target_link_libraries(unit_compute_point_on_spline 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_compute_point_on_spline
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_point_on_spline)
