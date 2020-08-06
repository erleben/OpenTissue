include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_jacobian_assembly src/unit_jacobian_assembly.cpp)

target_link_libraries(unit_jacobian_assembly 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_jacobian_assembly
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_jacobian_assembly)
