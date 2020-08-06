include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_projected_steepest_descent src/unit_projected_steepest_descent.cpp)

target_link_libraries(unit_projected_steepest_descent 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_projected_steepest_descent
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_projected_steepest_descent)

