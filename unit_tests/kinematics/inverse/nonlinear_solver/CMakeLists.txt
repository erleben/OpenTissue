include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_nonlinear_solver src/unit_nonlinear_solver.cpp)

target_link_libraries(unit_nonlinear_solver 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_nonlinear_solver
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_nonlinear_solver)
