INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_compute_index_reordering src/unit_compute_index_reordering.cpp)

TARGET_LINK_LIBRARIES(unit_compute_index_reordering ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_index_reordering
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_index_reordering unit_compute_index_reordering )

