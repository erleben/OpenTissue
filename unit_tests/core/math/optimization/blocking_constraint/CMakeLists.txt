include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_blocking_constraint src/unit_blocking_constraint.cpp)

target_link_libraries(unit_blocking_constraint 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_blocking_constraint
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_blocking_constraint)

