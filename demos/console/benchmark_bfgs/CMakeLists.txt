include_directories( ${PROJECT_SOURCE_DIR}/src )

add_executable(benchmark_bfgs src/benchmark_bfgs.cpp)

target_link_libraries(benchmark_bfgs
  PRIVATE
    OpenTissue
)

install(
  TARGETS benchmark_bfgs
  RUNTIME DESTINATION  bin/units
  COMPONENT Demos
  )

