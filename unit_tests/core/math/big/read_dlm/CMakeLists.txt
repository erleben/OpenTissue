INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/src )

ADD_EXECUTABLE(unit_read_dlm src/unit_read_dlm.cpp)

TARGET_LINK_LIBRARIES(unit_read_dlm ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_read_dlm
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_read_dlm unit_read_dlm )

