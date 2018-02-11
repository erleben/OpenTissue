INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_compute_weighted_diff src/unit_compute_weighted_diff.cpp)

TARGET_LINK_LIBRARIES(unit_compute_weighted_diff ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_weighted_diff
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_weighted_diff unit_compute_weighted_diff )
