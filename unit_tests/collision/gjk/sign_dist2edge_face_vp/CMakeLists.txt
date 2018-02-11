ADD_EXECUTABLE(unit_signed_distance_to_edge_face_vp src/unit_sign_dist2edge_face_vp.cpp)

TARGET_LINK_LIBRARIES(unit_signed_distance_to_edge_face_vp ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_signed_distance_to_edge_face_vp
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_signed_distance_to_edge_face_vp unit_signed_distance_to_edge_face_vp )

