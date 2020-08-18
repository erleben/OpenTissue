include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_symmetric_gauss_seidel src/unit_symmetric_gauss_seidel.cpp)

target_link_libraries(unit_symmetric_gauss_seidel 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_symmetric_gauss_seidel
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_symmetric_gauss_seidel)

