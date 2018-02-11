ADD_EXECUTABLE(unit_outside_triangle src/unit_outside_tri.cpp)

TARGET_LINK_LIBRARIES(unit_outside_triangle ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_outside_triangle
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_outside_triangle unit_outside_triangle)

