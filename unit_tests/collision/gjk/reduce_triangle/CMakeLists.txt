add_executable(unit_reduce_triangle src/unit_reduce_triangle.cpp)

target_link_libraries(unit_reduce_triangle
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_reduce_triangle
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_reduce_triangle)

