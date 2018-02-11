ADD_EXECUTABLE(dynamic_table_dispatcher src/dynamic_table_dispatcher.cpp)

INSTALL(
  TARGETS dynamic_table_dispatcher
  RUNTIME DESTINATION  bin/demos/console/
  )