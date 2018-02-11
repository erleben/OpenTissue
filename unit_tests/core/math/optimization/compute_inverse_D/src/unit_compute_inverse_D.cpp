//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/non_smooth_newton/optimization_compute_inverse_D.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_non_smooth_newton_compute_inverse_D);

BOOST_AUTO_TEST_CASE(test_case)
{
  typedef ublas::compressed_matrix<double> matrix_type;

  matrix_type D;
  matrix_type invD;
  matrix_type I;

  D.resize(6,6,false);
  invD.resize(6,6,false);
  I.resize(6,6,false);

  D(0,0) = 1.0;  D(0,1) = 0.0;  D(0,2) = 0.0;  D(0,3) = 0.0;  D(0,4) = 0.0;  D(0,5) = 0.0;
  D(1,0) = 2.0;  D(1,1) = 1.0;  D(1,2) = 0.0;  D(1,3) = 0.0;  D(1,4) = 0.0;  D(1,5) = 0.0;
  D(2,0) = 3.0;  D(2,1) = 0.0;  D(2,2) = 1.0;  D(2,3) = 0.0;  D(2,4) = 0.0;  D(2,5) = 0.0;
  D(3,0) = 0.0;  D(3,1) = 0.0;  D(3,2) = 0.0;  D(3,3) = 1.0;  D(3,4) = 0.0;  D(3,5) = 0.0;
  D(4,0) = 0.0;  D(4,1) = 0.0;  D(4,2) = 0.0;  D(4,3) = 4.0;  D(4,4) = 1.0;  D(4,5) = 0.0;
  D(5,0) = 0.0;  D(5,1) = 0.0;  D(5,2) = 0.0;  D(5,3) = 5.0;  D(5,4) = 0.0;  D(5,5) = 1.0;

  invD = D;
  OpenTissue::math::optimization::detail::compute_inverse_D( invD );

  ublas::sparse_prod(D, invD, I);

  double tol = 0.01;

  BOOST_CHECK_CLOSE( double( I(0,0) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(0,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(0,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(0,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(0,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(0,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(1,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(1,1) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(1,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(1,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(1,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(1,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(2,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(2,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(2,2) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(2,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(2,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(2,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(3,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(3,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(3,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(3,3) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(3,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(3,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(4,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(4,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(4,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(4,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(4,4) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(4,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(5,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(5,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(5,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(5,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(5,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( I(5,5) ), double( 1.0 ), tol );

}

BOOST_AUTO_TEST_SUITE_END();
