include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_prod src/unit_prod.cpp)

target_link_libraries(unit_prod 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)
install(
  TARGETS unit_prod
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_prod)

