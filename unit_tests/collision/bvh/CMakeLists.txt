add_executable(unit_bvh src/unit_bvh.cpp)

target_link_libraries(unit_bvh
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_bvh
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_bvh)

