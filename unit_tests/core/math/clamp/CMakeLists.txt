add_executable(unit_clamp src/unit_clamp.cpp)

target_link_libraries(unit_clamp 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_clamp
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_clamp)
