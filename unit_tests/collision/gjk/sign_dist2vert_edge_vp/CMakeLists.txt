add_executable(unit_signed_distance_to_vertex_edge_vp src/unit_sign_dist2vert_edge_vp.cpp)

target_link_libraries(unit_signed_distance_to_vertex_edge_vp
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_signed_distance_to_vertex_edge_vp
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_signed_distance_to_vertex_edge_vp)

