add_executable(unit_make_cubic_interpolation src/unit_make_cubic_interpolation.cpp)

target_link_libraries(unit_make_cubic_interpolation 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_make_cubic_interpolation
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_make_cubic_interpolation)
