add_executable(unit_compute_knot_span src/unit_compute_knot_span.cpp)

target_link_libraries(unit_compute_knot_span 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_compute_knot_span
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_knot_span)
