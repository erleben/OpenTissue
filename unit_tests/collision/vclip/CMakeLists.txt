ADD_EXECUTABLE(unit_vclip src/unit_vclip.cpp)

TARGET_LINK_LIBRARIES(unit_vclip ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_vclip
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_vclip unit_vclip )



