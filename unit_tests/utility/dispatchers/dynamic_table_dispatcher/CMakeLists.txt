ADD_EXECUTABLE(unit_dynamic_table_dispatcher src/unit_dynamic_table_dispatcher.cpp)

TARGET_LINK_LIBRARIES(unit_dynamic_table_dispatcher ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_dynamic_table_dispatcher
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_dynamic_table_dispatcher unit_dynamic_table_dispatcher )
