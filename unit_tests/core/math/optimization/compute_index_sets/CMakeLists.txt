INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_compute_index_sets src/unit_compute_index_sets.cpp)

TARGET_LINK_LIBRARIES(unit_compute_index_sets ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_index_sets
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_index_sets unit_compute_index_sets )

