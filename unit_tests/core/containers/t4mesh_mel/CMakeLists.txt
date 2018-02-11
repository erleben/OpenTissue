ADD_EXECUTABLE(unit_t4mesh_mel src/unit_t4mesh_mel.cpp)

TARGET_LINK_LIBRARIES(unit_t4mesh_mel ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_t4mesh_mel
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_t4mesh_mel unit_t4mesh_mel )


