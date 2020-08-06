include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_chain src/unit_chain.cpp)

target_link_libraries(unit_chain 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_chain
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_chain)
