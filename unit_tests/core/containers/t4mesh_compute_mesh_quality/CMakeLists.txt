ADD_EXECUTABLE(unit_t4mesh_compute_mesh_quality src/unit_t4mesh_compute_mesh_quality.cpp)

TARGET_LINK_LIBRARIES(unit_t4mesh_compute_mesh_quality ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_t4mesh_compute_mesh_quality
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_t4mesh_compute_mesh_quality unit_t4mesh_compute_mesh_quality )


