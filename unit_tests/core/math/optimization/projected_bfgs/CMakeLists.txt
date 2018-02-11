INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_projected_bfgs src/unit_projected_bfgs.cpp)

TARGET_LINK_LIBRARIES(unit_projected_bfgs ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_projected_bfgs
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_projected_bfgs unit_projected_bfgs )

