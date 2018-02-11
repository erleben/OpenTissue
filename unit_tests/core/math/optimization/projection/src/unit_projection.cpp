//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <OpenTissue/core/math/optimization/optimization_make_constant_bounds.h>
#include <OpenTissue/core/math/optimization/optimization_project.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

template<typename bound_function>
inline void test_projection( bound_function const & lower, bound_function const & upper, ublas::vector<double> const & x)
{
  double tol =0.01;
  {
    ublas::vector<double>  y;
    y = x;
    OpenTissue::math::optimization::project(x,lower,upper,y);
    BOOST_CHECK_CLOSE( y(0) , 1.0 , tol );
    BOOST_CHECK_CLOSE( y(1) , -2.0 , tol );
    BOOST_CHECK_CLOSE( y(2) , 3.0 , tol );
    BOOST_CHECK_CLOSE( y(3) , -4.0 , tol );
    BOOST_CHECK_CLOSE( y(4) , 5.0 , tol );
    BOOST_CHECK_CLOSE( y(5) , -6.0 , tol );
    BOOST_CHECK_CLOSE( y(6) , 7.0 , tol );
    BOOST_CHECK_CLOSE( y(7) , -8.0 , tol );
    BOOST_CHECK_CLOSE( y(8) , 9.0 , tol );
    BOOST_CHECK_CLOSE( y(9) , -10.0 , tol );
  }
  {
    ublas::vector<double>  y;
    y = x;

    OpenTissue::math::optimization::project(y,lower,upper);

    BOOST_CHECK_CLOSE( y(0) , 1.0 , tol );
    BOOST_CHECK_CLOSE( y(1) , -2.0 , tol );
    BOOST_CHECK_CLOSE( y(2) , 3.0 , tol );
    BOOST_CHECK_CLOSE( y(3) , -4.0 , tol );
    BOOST_CHECK_CLOSE( y(4) , 5.0 , tol );
    BOOST_CHECK_CLOSE( y(5) , -6.0 , tol );
    BOOST_CHECK_CLOSE( y(6) , 7.0 , tol );
    BOOST_CHECK_CLOSE( y(7) , -8.0 , tol );
    BOOST_CHECK_CLOSE( y(8) , 9.0 , tol );
    BOOST_CHECK_CLOSE( y(9) , -10.0 , tol );
  }
  {
    ublas::vector<double>  y;
    y = x;
    OpenTissue::math::optimization::Projection<double, bound_function> P(lower,upper);
    P(x,y);
    BOOST_CHECK_CLOSE( y(0) , 1.0 , tol );
    BOOST_CHECK_CLOSE( y(1) , -2.0 , tol );
    BOOST_CHECK_CLOSE( y(2) , 3.0 , tol );
    BOOST_CHECK_CLOSE( y(3) , -4.0 , tol );
    BOOST_CHECK_CLOSE( y(4) , 5.0 , tol );
    BOOST_CHECK_CLOSE( y(5) , -6.0 , tol );
    BOOST_CHECK_CLOSE( y(6) , 7.0 , tol );
    BOOST_CHECK_CLOSE( y(7) , -8.0 , tol );
    BOOST_CHECK_CLOSE( y(8) , 9.0 , tol );
    BOOST_CHECK_CLOSE( y(9) , -10.0 , tol );
  }
  {
    ublas::vector<double>  y;
    y = x;
    OpenTissue::math::optimization::Projection<double, bound_function> P(lower,upper);
    P(y);
    BOOST_CHECK_CLOSE( y(0) , 1.0 , tol );
    BOOST_CHECK_CLOSE( y(1) , -2.0 , tol );
    BOOST_CHECK_CLOSE( y(2) , 3.0 , tol );
    BOOST_CHECK_CLOSE( y(3) , -4.0 , tol );
    BOOST_CHECK_CLOSE( y(4) , 5.0 , tol );
    BOOST_CHECK_CLOSE( y(5) , -6.0 , tol );
    BOOST_CHECK_CLOSE( y(6) , 7.0 , tol );
    BOOST_CHECK_CLOSE( y(7) , -8.0 , tol );
    BOOST_CHECK_CLOSE( y(8) , 9.0 , tol );
    BOOST_CHECK_CLOSE( y(9) , -10.0 , tol );
  }
  {
    ublas::vector<double>  y;
    y = x;
    OpenTissue::math::optimization::NoProjection<double> P;
    P(x,y);
    BOOST_CHECK_CLOSE( y(0) , x(0) , tol );
    BOOST_CHECK_CLOSE( y(1) , x(1) , tol );
    BOOST_CHECK_CLOSE( y(2) , x(2) , tol );
    BOOST_CHECK_CLOSE( y(3) , x(3) , tol );
    BOOST_CHECK_CLOSE( y(4) , x(4) , tol );
    BOOST_CHECK_CLOSE( y(5) , x(5) , tol );
    BOOST_CHECK_CLOSE( y(6) , x(6) , tol );
    BOOST_CHECK_CLOSE( y(7) , x(7) , tol );
    BOOST_CHECK_CLOSE( y(8) , x(8) , tol );
    BOOST_CHECK_CLOSE( y(9) , x(9) , tol );
  }
  {
    ublas::vector<double>  y;
    y = x;
    OpenTissue::math::optimization::NoProjection<double> P;
    P(y);
    BOOST_CHECK_CLOSE( y(0) , x(0) , tol );
    BOOST_CHECK_CLOSE( y(1) , x(1) , tol );
    BOOST_CHECK_CLOSE( y(2) , x(2) , tol );
    BOOST_CHECK_CLOSE( y(3) , x(3) , tol );
    BOOST_CHECK_CLOSE( y(4) , x(4) , tol );
    BOOST_CHECK_CLOSE( y(5) , x(5) , tol );
    BOOST_CHECK_CLOSE( y(6) , x(6) , tol );
    BOOST_CHECK_CLOSE( y(7) , x(7) , tol );
    BOOST_CHECK_CLOSE( y(8) , x(8) , tol );
    BOOST_CHECK_CLOSE( y(9) , x(9) , tol );
  }
}


BOOST_AUTO_TEST_SUITE(opentissue_math_big_projection);

BOOST_AUTO_TEST_CASE(test_case)
{
  ublas::vector<double> lower;
  ublas::vector<double> upper;
  ublas::vector<double> x;

  lower.resize(10,false);
  upper.resize(10,false);
  x.resize(10,false);

  upper(0) = 1.0;
  upper(1) = 2.0;
  upper(2) = 3.0;
  upper(3) = 4.0;
  upper(4) = 5.0;
  upper(5) = 6.0;
  upper(6) = 7.0;
  upper(7) = 8.0;
  upper(8) = 9.0;
  upper(9) = 10.0;

  lower = -upper;

  x(0) =  2.0;
  x(1) = -3.0;
  x(2) =  4.0;
  x(3) = -5.0;
  x(4) =  6.0;
  x(5) = -7.0;
  x(6) =  8.0;
  x(7) = -9.0;
  x(8) =  10.0;
  x(9) = -11.0;

  test_projection( 
    OpenTissue::math::optimization::make_constant_bounds( lower )
    , OpenTissue::math::optimization::make_constant_bounds( upper )
    , x
    );

}

BOOST_AUTO_TEST_SUITE_END();
