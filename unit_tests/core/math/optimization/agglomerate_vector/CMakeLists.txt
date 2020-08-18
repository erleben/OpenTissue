include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_agglomerate_vector src/unit_agglomerate_vector.cpp)

target_link_libraries(unit_agglomerate_vector 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_agglomerate_vector
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_agglomerate_vector)

