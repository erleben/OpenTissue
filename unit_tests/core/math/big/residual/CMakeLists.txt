include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_residual src/unit_residual.cpp)

target_link_libraries(unit_residual 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_residual
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_residual)

