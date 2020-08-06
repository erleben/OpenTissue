add_executable(unit_axis_angle src/unit_axis_angle.cpp)

target_link_libraries(unit_axis_angle 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_axis_angle
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_axis_angle)
