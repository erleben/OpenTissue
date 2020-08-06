include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(unit_lu src/unit_lu.cpp)

target_link_libraries(unit_lu 
  PRIVATE
    Boost::unit_test_framework
    OpenTissue
)

install(
  TARGETS unit_lu
  RUNTIME DESTINATION  bin/units
  )

ot_add_test(unit_lu)

