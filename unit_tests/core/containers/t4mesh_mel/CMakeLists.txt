add_executable(unit_t4mesh_mel src/unit_t4mesh_mel.cpp)

target_link_libraries(unit_t4mesh_mel
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_t4mesh_mel
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_t4mesh_mel)


