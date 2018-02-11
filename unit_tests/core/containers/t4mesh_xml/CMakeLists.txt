ADD_EXECUTABLE(unit_t4mesh_xml src/unit_t4mesh_xml.cpp)

TARGET_LINK_LIBRARIES(unit_t4mesh_xml ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_t4mesh_xml
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_t4mesh_xml unit_t4mesh_xml )


