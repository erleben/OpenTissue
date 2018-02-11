INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_shur_system src/unit_shur_system.cpp)

TARGET_LINK_LIBRARIES(unit_shur_system ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_shur_system
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_shur_system unit_shur_system )

