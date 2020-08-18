include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_prod_row src/unit_prod_row.cpp)

target_link_libraries(unit_prod_row 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_prod_row
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_prod_row)

