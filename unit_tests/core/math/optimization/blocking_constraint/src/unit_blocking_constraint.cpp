//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <OpenTissue/core/math/optimization/optimization_blocking_constraint_search.h>
#include <OpenTissue/core/math/optimization/optimization_make_constant_bounds.h>
#include <OpenTissue/core/math/optimization/optimization_bounds2constraint.h>


#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_blocking_constraint);

BOOST_AUTO_TEST_CASE(compile_test_case)
{
  double tau = 0.0;
  size_t blocking_idx = 1000;

  ublas::vector<double> x;
  ublas::vector<double> p;
  ublas::vector<double> values;

  x.resize(3,false);
  p.resize(3,false);
  values.resize(3,false);

  values(0) = 1.0;
  values(1) = 2.0;
  values(2) = 3.0;
  x(0) = 4.0;
  x(1) = 4.0;
  x(2) = 4.0;
  p(0) = 0.0;
  p(1) = -4.0;
  p(2) = 0.0;

  bool is_blocked = OpenTissue::math::optimization::blocking_constraint_search( 
    x
    , p
    , OpenTissue::math::optimization::make_constraint_from_lower_bounds( OpenTissue::math::optimization::make_constant_bounds( values ) )
    , tau 
    , blocking_idx
    );

  BOOST_CHECK(is_blocked);
  BOOST_CHECK(blocking_idx == 1);
  BOOST_CHECK_CLOSE( tau, 0.5, 0.1 );

  values(0) = -2.0;
  values(1) = -2.0;
  values(2) = -2.0;
  x(0) = 0.0;
  x(1) = 0.0;
  x(2) = 0.0;
  p(0) = -1.0;
  p(1) = -1.0;
  p(2) = -1.0;

  is_blocked = OpenTissue::math::optimization::blocking_constraint_search( 
    x
    , p
    , OpenTissue::math::optimization::make_constraint_from_lower_bounds( OpenTissue::math::optimization::make_constant_bounds( values ) )
    , tau 
    , blocking_idx
    );

  BOOST_CHECK(!is_blocked);
  BOOST_CHECK(blocking_idx == 4);
  BOOST_CHECK_CLOSE( tau, 1.0, 0.1 );

  values(0) = 2.0;
  values(1) = 2.0;
  values(2) = 2.0;
  x(0) = 0.0;
  x(1) = 0.0;
  x(2) = 0.0;
  p(0) = 1.0;
  p(1) = 4.0;
  p(2) = 1.0;

  is_blocked = OpenTissue::math::optimization::blocking_constraint_search( 
    x
    , p
    , OpenTissue::math::optimization::make_constraint_from_upper_bounds(   OpenTissue::math::optimization::make_constant_bounds( values ) )
    , tau 
    , blocking_idx
    );

  BOOST_CHECK(is_blocked);
  BOOST_CHECK(blocking_idx == 1);
  BOOST_CHECK_CLOSE( tau, 0.5, 0.1 );
}

BOOST_AUTO_TEST_SUITE_END();
