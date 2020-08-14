//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_timer.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#if defined(WIN32)
#  include <winbase.h>  // for Sleep()
#else
#  include <unistd.h>  // for sleep()
#endif

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_utility_timer);

  BOOST_AUTO_TEST_CASE(double_testing)
  {
    OpenTissue::utility::Timer<double> timer;
    timer.start();
#if defined(WIN32)
    Sleep( 2 * 1000 );
#else
    sleep ( 2 );
#endif
    timer.stop();
    double duration = timer();
    BOOST_CHECK( duration > 1.9);
    BOOST_CHECK( duration < 2.1);
  }

BOOST_AUTO_TEST_SUITE_END();
