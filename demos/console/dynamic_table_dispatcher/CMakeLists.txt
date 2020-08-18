add_executable(dynamic_table_dispatcher src/dynamic_table_dispatcher.cpp)

target_link_libraries(dynamic_table_dispatcher
  PRIVATE
    OpenTissue
)

install(
  TARGETS dynamic_table_dispatcher
  RUNTIME DESTINATION  bin/demos/console/
  COMPONENT Demos
  )
