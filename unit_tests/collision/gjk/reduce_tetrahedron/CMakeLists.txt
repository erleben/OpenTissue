ADD_EXECUTABLE(unit_reduce_tetrahedron src/unit_reduce_tetrahedron.cpp)

TARGET_LINK_LIBRARIES(unit_reduce_tetrahedron ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_reduce_tetrahedron
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_reduce_tetrahedron unit_reduce_tetrahedron )

