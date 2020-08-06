add_executable(unit_compute_basis_derivatives src/unit_comp_basis_deriv.cpp)

target_link_libraries(unit_compute_basis_derivatives 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_compute_basis_derivatives
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_basis_derivatives)
