ADD_EXECUTABLE(unit_signed_distance_to_triangle src/unit_sign_dist2tri.cpp)

TARGET_LINK_LIBRARIES(unit_signed_distance_to_triangle ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_signed_distance_to_triangle
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_signed_distance_to_triangle unit_signed_distance_to_triangle)

