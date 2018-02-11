INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_compute_partitioned_jacobian src/unit_compute_partitioned_jacobian.cpp)

TARGET_LINK_LIBRARIES(unit_compute_partitioned_jacobian ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_partitioned_jacobian
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_partitioned_jacobian unit_compute_partitioned_jacobian )

