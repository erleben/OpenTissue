INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_compute_generalized_minimal_map src/unit_comp_gen_min_map.cpp)

TARGET_LINK_LIBRARIES(unit_compute_generalized_minimal_map ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_generalized_minimal_map
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_generalized_minimal_map unit_compute_generalized_minimal_map )

