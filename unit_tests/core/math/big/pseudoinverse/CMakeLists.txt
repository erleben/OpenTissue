INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_pseudoinverse src/unit_pseudoinverse.cpp)

TARGET_LINK_LIBRARIES(unit_pseudoinverse ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_pseudoinverse
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_pseudoinverse unit_pseudoinverse )

