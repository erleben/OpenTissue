add_executable(unit_compute_nonzero_basis src/unit_compute_nonzero_basis.cpp)

target_link_libraries(unit_compute_nonzero_basis 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_compute_nonzero_basis
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_compute_nonzero_basis)
