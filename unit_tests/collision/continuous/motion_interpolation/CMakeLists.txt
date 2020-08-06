add_executable(unit_motion_interpolation src/unit_motion_interpolation.cpp)

target_link_libraries(unit_motion_interpolation
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_motion_interpolation
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_motion_interpolation)

