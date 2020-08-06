add_executable(unit_ray_aabb src/unit_ray_aabb.cpp)

target_link_libraries(unit_ray_aabb
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_ray_aabb
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_ray_aabb)



