add_executable(unit_gjk_constants src/unit_constants.cpp)

target_link_libraries(unit_gjk_constants
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_gjk_constants
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_gjk_constants)

