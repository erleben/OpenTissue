INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_projected_steepest_descent src/unit_projected_steepest_descent.cpp)

TARGET_LINK_LIBRARIES(unit_projected_steepest_descent ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_projected_steepest_descent
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_projected_steepest_descent unit_projected_steepest_descent )

