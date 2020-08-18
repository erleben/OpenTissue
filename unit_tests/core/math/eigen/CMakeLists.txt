add_executable(unit_eigen src/unit_eigen.cpp)

target_link_libraries(unit_eigen 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_eigen
  RUNTIME DESTINATION  bin/units
  )


ot_add_test(unit_eigen)
