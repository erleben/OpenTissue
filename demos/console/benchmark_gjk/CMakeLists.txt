INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(benchmark_gjk src/benchmark_gjk.cpp)

TARGET_LINK_LIBRARIES(benchmark_gjk ${OPENTISSUE_LIBS} )

INSTALL(
  TARGETS benchmark_gjk
  RUNTIME DESTINATION  bin/units
  )

