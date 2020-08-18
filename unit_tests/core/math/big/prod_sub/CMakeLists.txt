include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_prod_sub src/unit_prod_sub.cpp)

target_link_libraries(unit_prod_sub 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_prod_sub
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_prod_sub)


