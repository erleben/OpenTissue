ADD_EXECUTABLE(unit_polymesh_compute_voronoi src/unit_polymesh_compute_voronoi.cpp)

TARGET_LINK_LIBRARIES(unit_polymesh_compute_voronoi ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_polymesh_compute_voronoi
  RUNTIME DESTINATION  bin/units
  )


ADD_TEST( unit_polymesh_compute_voronoi unit_polymesh_compute_voronoi )
