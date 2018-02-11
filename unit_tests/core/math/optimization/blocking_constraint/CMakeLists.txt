INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_blocking_constraint src/unit_blocking_constraint.cpp)

TARGET_LINK_LIBRARIES(unit_blocking_constraint ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_blocking_constraint
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_blocking_constraint unit_blocking_constraint )

