include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(benchmark_gjk src/benchmark_gjk.cpp)

target_link_libraries(benchmark_gjk
  PRIVATE
    OpenTissue
)

install(
  TARGETS benchmark_gjk
  RUNTIME DESTINATION  bin/units
  COMPONENT Demos
  )

