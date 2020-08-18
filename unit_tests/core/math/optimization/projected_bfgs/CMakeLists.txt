include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_projected_bfgs src/unit_projected_bfgs.cpp)

target_link_libraries(unit_projected_bfgs 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_projected_bfgs
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_projected_bfgs)

