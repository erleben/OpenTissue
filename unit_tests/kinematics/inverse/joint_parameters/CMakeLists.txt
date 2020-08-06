include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_joint_parameters src/unit_joint_parameters.cpp)

target_link_libraries(unit_joint_parameters 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_joint_parameters
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_joint_parameters)
