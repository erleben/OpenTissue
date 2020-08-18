add_executable(unit_outside_edge_face src/unit_outside_edge_face.cpp)

target_link_libraries(unit_outside_edge_face
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_outside_edge_face
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_outside_edge_face)

