ADD_EXECUTABLE(unit_nubs src/unit_nubs.cpp)

TARGET_LINK_LIBRARIES(unit_nubs ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_nubs 
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_nubs unit_nubs )