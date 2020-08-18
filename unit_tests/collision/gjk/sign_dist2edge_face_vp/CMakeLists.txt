add_executable(unit_signed_distance_to_edge_face_vp src/unit_sign_dist2edge_face_vp.cpp)

target_link_libraries(unit_signed_distance_to_edge_face_vp
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_signed_distance_to_edge_face_vp
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_signed_distance_to_edge_face_vp)

