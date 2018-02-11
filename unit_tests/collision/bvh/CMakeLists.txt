ADD_EXECUTABLE(unit_bvh src/unit_bvh.cpp)

TARGET_LINK_LIBRARIES(unit_bvh ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_bvh
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_bvh unit_bvh )

