INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(benchmark_bfgs src/benchmark_bfgs.cpp)

TARGET_LINK_LIBRARIES(benchmark_bfgs ${OPENTISSUE_LIBS} )

INSTALL(
  TARGETS benchmark_bfgs
  RUNTIME DESTINATION  bin/units
  )

