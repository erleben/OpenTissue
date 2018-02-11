//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/optimization/optimization_armijo_backtracking.h>

#include <cmath> // needed for std::cos

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


double f( double const & x)
{
  using std::cos;

  return cos( x );
}


double df( double const & x )
{
  using std::sin;

  return -sin( x );
}


class F
{
public:
  double operator()( double const & x ) const  {    return f(x);  }
};


class dF
{
public:
  double operator()( double const & x ) const  {    return df(x);  }
};

BOOST_AUTO_TEST_SUITE(opentissue_math_big_armijo_backtracking);

BOOST_AUTO_TEST_CASE(no_test_case)
{

}

BOOST_AUTO_TEST_SUITE_END();
