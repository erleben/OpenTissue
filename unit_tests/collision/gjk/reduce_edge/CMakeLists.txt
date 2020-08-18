add_executable(unit_reduce_edge src/unit_reduce_edge.cpp)

target_link_libraries(unit_reduce_edge
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_reduce_edge
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_reduce_edge)

