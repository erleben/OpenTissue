add_executable(unit_outside_vertex_edge src/unit_outside_vertex_edge.cpp)

target_link_libraries(unit_outside_vertex_edge
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_outside_vertex_edge
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_outside_vertex_edge)

