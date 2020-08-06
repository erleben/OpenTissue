include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(benchmark_svd src/benchmark_svd.cpp)

target_link_libraries(benchmark_svd
  PRIVATE
    OpenTissue
)

install(
  TARGETS benchmark_svd
  RUNTIME DESTINATION  bin/units
  COMPONENT Demos
  )

