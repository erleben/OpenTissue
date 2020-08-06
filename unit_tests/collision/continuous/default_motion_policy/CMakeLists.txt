add_executable(unit_default_motion_policy src/unit_default_motion_policy.cpp)

target_link_libraries(unit_default_motion_policy
  PRIVATE
      Boost::unit_test_framework
      OpenTissue
)

install(
  TARGETS unit_default_motion_policy
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_default_motion_policy)

