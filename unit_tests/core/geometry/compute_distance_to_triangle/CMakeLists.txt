ADD_EXECUTABLE(unit_compute_distance_to_triangle src/unit_compute_distance_to_triangle.cpp)

TARGET_LINK_LIBRARIES(unit_compute_distance_to_triangle ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_compute_distance_to_triangle
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_compute_distance_to_triangle unit_compute_distance_to_triangle  )
