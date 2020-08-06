include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_prod_trans src/unit_prod_trans.cpp)

target_link_libraries(unit_prod_trans 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_prod_trans
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_prod_trans)

