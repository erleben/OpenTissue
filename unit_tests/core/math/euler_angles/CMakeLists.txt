add_executable(unit_euler_angles src/unit_euler_angles.cpp)

target_link_libraries(unit_euler_angles 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_euler_angles
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_euler_angles)
