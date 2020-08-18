include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_cholesky src/unit_cholesky.cpp)

target_link_libraries(unit_cholesky 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_cholesky
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_cholesky)

