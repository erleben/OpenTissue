//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_get_environment_variable.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_utility_get_environment_variable);

BOOST_AUTO_TEST_CASE(simple_test)
{
  std::string path;
  BOOST_CHECK_NO_THROW( path = OpenTissue::utility::get_environment_variable("OPENTISSUE") );
  BOOST_CHECK( !path.empty() );
  BOOST_CHECK_THROW(OpenTissue::utility::get_environment_variable("GARBAGE_NON_EXISTING_VARIABLE"), std::logic_error );
}

BOOST_AUTO_TEST_SUITE_END();
