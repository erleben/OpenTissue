add_executable(unit_polymesh_is_point_inside src/unit_polymesh_is_point_inside.cpp)

target_link_libraries(unit_polymesh_is_point_inside
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_polymesh_is_point_inside
  RUNTIME DESTINATION  bin/units
  )


ot_add_test(unit_polymesh_is_point_inside)
