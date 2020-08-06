add_executable(unit_voronoi_policy src/unit_voronoi_policy.cpp)

target_link_libraries(unit_voronoi_policy
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_voronoi_policy
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_voronoi_policy)

