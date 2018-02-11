INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(benchmark_svd src/benchmark_svd.cpp)

TARGET_LINK_LIBRARIES(benchmark_svd ${OPENTISSUE_LIBS} )

INSTALL(
  TARGETS benchmark_svd
  RUNTIME DESTINATION  bin/units
  )

