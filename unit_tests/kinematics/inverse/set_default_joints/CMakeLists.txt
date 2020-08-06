include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_set_default_joints src/unit_set_default_joints.cpp)

target_link_libraries(unit_set_default_joints 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_set_default_joints
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_set_default_joints)
