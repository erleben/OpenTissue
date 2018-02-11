//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/gjk/gjk_constants.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_constants);

BOOST_AUTO_TEST_CASE(simple_testing)
{
  std::string msg = "";

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::ABSOLUTE_CONVERGENCE ) );
  BOOST_CHECK( msg == "Absolute convergence test passed" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::EXCEEDED_MAX_ITERATIONS_LIMIT ) );
  BOOST_CHECK( msg == "Maximum iteration limit was exceeded" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::INTERSECTION ) );
  BOOST_CHECK( msg == "Intersection was found" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::ITERATING ) );
  BOOST_CHECK( msg == "Unexpected termination while iterating" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::NON_DESCEND_DIRECTION ) );
  BOOST_CHECK( msg == "Non descent direction was encountered" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::RELATIVE_CONVERGENCE ) );
  BOOST_CHECK( msg == "Relative convergence test passed" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::STAGNATION ) );
  BOOST_CHECK( msg == "Stagnation test passed" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( OpenTissue::gjk::SIMPLEX_EXPANSION_FAILED ) );
  BOOST_CHECK( msg == "Simplex expansion failure" );

  BOOST_CHECK_NO_THROW( msg = OpenTissue::gjk::get_status_message( 7654u ) );
  BOOST_CHECK( msg == "unrecognised error" );

}

BOOST_AUTO_TEST_SUITE_END();
