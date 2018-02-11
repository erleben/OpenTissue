ADD_EXECUTABLE(unit_compute_nonzero_basis src/unit_compute_nonzero_basis.cpp)

TARGET_LINK_LIBRARIES(unit_compute_nonzero_basis ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_nonzero_basis
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_nonzero_basis unit_compute_nonzero_basis )