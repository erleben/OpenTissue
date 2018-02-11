ADD_EXECUTABLE(unit_init_m_table src/unit_init_m_table.cpp)

TARGET_LINK_LIBRARIES(unit_init_m_table ${OPENTISSUE_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

INSTALL(
  TARGETS unit_init_m_table
  RUNTIME DESTINATION  bin/units
  )

ADD_TEST( unit_init_m_table unit_init_m_table )