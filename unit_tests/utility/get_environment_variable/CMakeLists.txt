add_executable(unit_get_environment_variable src/unit_get_environment_variable.cpp)

target_link_libraries(unit_get_environment_variable
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_get_environment_variable
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_get_environment_variable)
