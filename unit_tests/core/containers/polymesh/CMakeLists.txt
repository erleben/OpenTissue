ADD_EXECUTABLE(unit_polymesh src/unit_polymesh.cpp)

TARGET_LINK_LIBRARIES(unit_polymesh ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_polymesh 
  RUNTIME DESTINATION  bin/units
  )


ADD_TEST( unit_polymesh unit_polymesh )
