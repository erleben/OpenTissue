ADD_EXECUTABLE(unit_polymesh_is_point_inside src/unit_polymesh_is_point_inside.cpp)

TARGET_LINK_LIBRARIES(unit_polymesh_is_point_inside ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_polymesh_is_point_inside 
  RUNTIME DESTINATION  bin/units
  )


ADD_TEST( unit_polymesh_is_point_inside unit_polymesh_is_point_inside )
