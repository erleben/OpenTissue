INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_const_bounds src/unit_const_bounds.cpp)

TARGET_LINK_LIBRARIES(unit_const_bounds ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_const_bounds
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_const_bounds unit_const_bounds )

