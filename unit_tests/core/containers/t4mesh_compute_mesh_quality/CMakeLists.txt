add_executable(unit_t4mesh_compute_mesh_quality src/unit_t4mesh_compute_mesh_quality.cpp)

target_link_libraries(unit_t4mesh_compute_mesh_quality
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_t4mesh_compute_mesh_quality
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_t4mesh_compute_mesh_quality)


