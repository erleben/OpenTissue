ADD_EXECUTABLE(unit_voronoi_policy src/unit_voronoi_policy.cpp)

TARGET_LINK_LIBRARIES(unit_voronoi_policy ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_voronoi_policy
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_voronoi_policy unit_voronoi_policy )

