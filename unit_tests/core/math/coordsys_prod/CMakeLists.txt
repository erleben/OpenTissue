ADD_EXECUTABLE(unit_coordsys_prod src/unit_coordsys_prod.cpp)

TARGET_LINK_LIBRARIES(unit_coordsys_prod ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_coordsys_prod
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_coordsys_prod unit_coordsys_prod )