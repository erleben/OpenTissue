include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_armijo src/unit_armijo.cpp)

target_link_libraries(unit_armijo 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_armijo
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_armijo)

