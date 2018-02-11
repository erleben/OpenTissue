INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_partition_vector src/unit_partition_vector.cpp)

TARGET_LINK_LIBRARIES(unit_partition_vector ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_partition_vector
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_partition_vector unit_partition_vector )

