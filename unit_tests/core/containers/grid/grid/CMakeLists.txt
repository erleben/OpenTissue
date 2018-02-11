ADD_EXECUTABLE(unit_grid src/unit_grid.cpp)

TARGET_LINK_LIBRARIES(unit_grid ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_grid
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_grid unit_grid )
