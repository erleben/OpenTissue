include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_prod_add_rhs src/unit_prod_add_rhs.cpp)

target_link_libraries(unit_prod_add_rhs 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_prod_add_rhs
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_prod_add_rhs)

