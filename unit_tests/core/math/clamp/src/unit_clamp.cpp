//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_functions.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_math_clamp);

  BOOST_AUTO_TEST_CASE(double_testing)
  {
    double const min_val = 0.1;
    double const max_val = 2.9;
    double clamped_val;

    // normal clamp tests
    clamped_val = math::clamp(1.5, min_val, max_val);
    BOOST_CHECK( clamped_val == 1.5 );

    clamped_val = math::clamp(-1.5, min_val, max_val);
    BOOST_CHECK( clamped_val == min_val );

    clamped_val = math::clamp(3.0, min_val, max_val);
    BOOST_CHECK( clamped_val == max_val );


    // clamp minimum tests
    clamped_val = math::clamp_min(1.5, min_val);
    BOOST_CHECK( clamped_val == 1.5 );

    clamped_val = math::clamp_min(-1.5, min_val);
    BOOST_CHECK( clamped_val == min_val );

    clamped_val = math::clamp_min(3.0, min_val);
    BOOST_CHECK( clamped_val == 3.0 );


    // clamp maximum tests
    clamped_val = math::clamp_max(1.5, max_val);
    BOOST_CHECK( clamped_val == 1.5 );

    clamped_val = math::clamp_max(-1.5, max_val);
    BOOST_CHECK( clamped_val == -1.5 );

    clamped_val = math::clamp_max(3.0, max_val);
    BOOST_CHECK( clamped_val == max_val );


    // clamp zero..one tests
    clamped_val = math::clamp_zero_one(0.5);
    BOOST_CHECK( clamped_val == 0.5 );

    clamped_val = math::clamp_zero_one(-1.5);
    BOOST_CHECK( clamped_val == 0.0 );

    clamped_val = math::clamp_zero_one(3.0);
    BOOST_CHECK( clamped_val == 1.0 );
  }


  BOOST_AUTO_TEST_CASE(float_testing)
  {
    float const min_val = 0.1f;
    float const max_val = 2.9f;
    float clamped_val;

    // normal clamp tests
    clamped_val = math::clamp(1.5f, min_val, max_val);
    BOOST_CHECK( clamped_val == 1.5f );

    clamped_val = math::clamp(-1.5f, min_val, max_val);
    BOOST_CHECK( clamped_val == min_val );

    clamped_val = math::clamp(3.0f, min_val, max_val);
    BOOST_CHECK( clamped_val == max_val );


    // clamp minimum tests
    clamped_val = math::clamp_min(1.5f, min_val);
    BOOST_CHECK( clamped_val == 1.5f );

    clamped_val = math::clamp_min(-1.5f, min_val);
    BOOST_CHECK( clamped_val == min_val );

    clamped_val = math::clamp_min(3.0f, min_val);
    BOOST_CHECK( clamped_val == 3.0f );


    // clamp maximum tests
    clamped_val = math::clamp_max(1.5f, max_val);
    BOOST_CHECK( clamped_val == 1.5f );

    clamped_val = math::clamp_max(-1.5f, max_val);
    BOOST_CHECK( clamped_val == -1.5f );

    clamped_val = math::clamp_max(3.0f, max_val);
    BOOST_CHECK( clamped_val == max_val );


    // clamp zero..one tests
    clamped_val = math::clamp_zero_one(0.5f);
    BOOST_CHECK( clamped_val == 0.5f );

    clamped_val = math::clamp_zero_one(-1.5f);
    BOOST_CHECK( clamped_val == 0.0f );

    clamped_val = math::clamp_zero_one(3.0f);
    BOOST_CHECK( clamped_val == 1.0f );
  }


BOOST_AUTO_TEST_SUITE_END();
