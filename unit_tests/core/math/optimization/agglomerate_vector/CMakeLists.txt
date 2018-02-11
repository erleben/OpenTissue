INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_agglomerate_vector src/unit_agglomerate_vector.cpp)

TARGET_LINK_LIBRARIES(unit_agglomerate_vector ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_agglomerate_vector
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_agglomerate_vector unit_agglomerate_vector )

