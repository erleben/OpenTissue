ADD_EXECUTABLE(unit_tag_traits src/unit_tag_traits.cpp)

TARGET_LINK_LIBRARIES(unit_tag_traits ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_tag_traits 
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_tag_traits unit_tag_traits )
