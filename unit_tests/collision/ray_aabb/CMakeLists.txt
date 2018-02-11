ADD_EXECUTABLE(unit_ray_aabb src/unit_ray_aabb.cpp)

TARGET_LINK_LIBRARIES(unit_ray_aabb ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_ray_aabb
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_ray_aabb unit_ray_aabb )



