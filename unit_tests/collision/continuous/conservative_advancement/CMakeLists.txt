add_executable(unit_conservative_advancement src/unit_conservative_advancement.cpp)

target_link_libraries(unit_conservative_advancement
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_conservative_advancement
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_conservative_advancement)

