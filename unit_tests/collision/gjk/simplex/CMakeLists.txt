add_executable(unit_simplex src/unit_simplex.cpp)

target_link_libraries(unit_simplex
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_simplex
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_simplex)

