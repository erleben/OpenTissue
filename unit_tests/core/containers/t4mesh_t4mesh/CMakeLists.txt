ADD_EXECUTABLE(unit_t4mesh_t4mesh src/unit_t4mesh_t4mesh.cpp)

TARGET_LINK_LIBRARIES(unit_t4mesh_t4mesh ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_t4mesh_t4mesh
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_t4mesh_t4mesh unit_t4mesh_t4mesh )


