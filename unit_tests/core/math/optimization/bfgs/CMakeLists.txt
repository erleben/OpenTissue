include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_bfgs src/unit_bfgs.cpp)

target_link_libraries(unit_bfgs 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_bfgs
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_bfgs)

