ADD_EXECUTABLE(unit_compute_basis src/unit_compute_basis.cpp)

TARGET_LINK_LIBRARIES(unit_compute_basis ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_basis
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_basis unit_compute_basis )