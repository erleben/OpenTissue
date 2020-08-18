add_executable(unit_t4mesh_t4mesh src/unit_t4mesh_t4mesh.cpp)

target_link_libraries(unit_t4mesh_t4mesh
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_t4mesh_t4mesh
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_t4mesh_t4mesh)


