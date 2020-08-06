add_executable(unit_reduce_tetrahedron src/unit_reduce_tetrahedron.cpp)

target_link_libraries(unit_reduce_tetrahedron
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_reduce_tetrahedron
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_reduce_tetrahedron)

