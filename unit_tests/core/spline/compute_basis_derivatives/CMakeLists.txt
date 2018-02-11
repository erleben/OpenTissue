ADD_EXECUTABLE(unit_compute_basis_derivatives src/unit_comp_basis_deriv.cpp)

TARGET_LINK_LIBRARIES(unit_compute_basis_derivatives ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_basis_derivatives
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_basis_derivatives unit_compute_basis_derivatives )