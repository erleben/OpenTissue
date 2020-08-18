add_executable(unit_plane_box src/unit_plane_box.cpp)

target_link_libraries(unit_plane_box
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_plane_box
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_plane_box)
