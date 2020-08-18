add_executable(unit_compute_derivatives src/unit_compute_derivatives.cpp)

target_link_libraries(unit_compute_derivatives 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_compute_derivatives
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_derivatives)
