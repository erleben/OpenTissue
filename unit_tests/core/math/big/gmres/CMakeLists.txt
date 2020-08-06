include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_gmres src/unit_gmres.cpp)

target_link_libraries(unit_gmres 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_gmres
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_gmres)

