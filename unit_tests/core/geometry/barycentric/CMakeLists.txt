add_executable(unit_barycentric src/unit_barycentric.cpp)

target_link_libraries(unit_barycentric
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_barycentric
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_barycentric)
