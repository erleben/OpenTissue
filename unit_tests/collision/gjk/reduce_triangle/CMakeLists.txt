ADD_EXECUTABLE(unit_reduce_triangle src/unit_reduce_triangle.cpp)

TARGET_LINK_LIBRARIES(unit_reduce_triangle ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_reduce_triangle
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_reduce_triangle unit_reduce_triangle )

