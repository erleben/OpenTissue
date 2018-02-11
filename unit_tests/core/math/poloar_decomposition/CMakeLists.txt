ADD_EXECUTABLE(unit_polar_decomposition src/unit_poloar_decomposition.cpp)

TARGET_LINK_LIBRARIES(unit_polar_decomposition ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_polar_decomposition
  RUNTIME DESTINATION  bin/units
  )


ADD_TEST( unit_polar_decomposition unit_polar_decomposition )
