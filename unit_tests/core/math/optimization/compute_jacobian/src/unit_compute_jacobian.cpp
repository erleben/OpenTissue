//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>


#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_compute_index_reordering.h>
#include <OpenTissue/core/math/optimization/optimization_make_mbd_bounds.h>
#include <OpenTissue/core/math/optimization/non_smooth_newton/optimization_compute_jacobian.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_non_smooth_newton_compute_full_jacobian);

BOOST_AUTO_TEST_CASE(test_case)
{
  typedef ublas::vector<size_t>   idx_vector_type;
  typedef ublas::vector<double>   vector_type;
  typedef ublas::compressed_matrix<double>   matrix_type;
  typedef OpenTissue::math::ValueTraits<double> value_traits;
  typedef double real_type;
  typedef size_t size_type;

  matrix_type A;
  vector_type mu,lo,hi;
  A.resize(10,10,false);
  mu.resize(10,false);
  lo.resize(10,false);
  hi.resize(10,false);

  OpenTissue::math::Random<double> value(0.0,1.0);
  for(size_t i=0;i<A.size1();++i)
  {
    mu(i) = value();
    lo(i) = value_traits::zero();
    hi(i) = value_traits::infinity();
    for(size_t j=0;j<A.size2();++j)
      A(i,j) = value();
  }

  idx_vector_type bitmask;

  bitmask.resize(10,false);

  static size_t const in_lower  = 1;
  static size_t const in_upper  = 2;
  static size_t const in_active = 4;

  bitmask(0) = in_upper;   // pi = inf
  bitmask(1) = in_lower;   // pi = 0
  bitmask(2) = in_active;  // pi = 0
  bitmask(3) = in_lower;   // pi = inf
  bitmask(4) = in_active;  // pi = 3
  bitmask(5) = in_upper;   // pi = 3
  bitmask(6) = in_active;  // pi = inf
  bitmask(7) = in_upper;   // pi = 6
  bitmask(8) = in_lower;   // pi = 6
  bitmask(9) = in_active;  // pi = inf

  idx_vector_type old2new;
  idx_vector_type new2old;

  OpenTissue::math::optimization::compute_index_reordering( bitmask, old2new, new2old );

  idx_vector_type pi;
  pi.resize(10,false);
  size_type nodep = OpenTissue::math::detail::highest<size_t>();
  pi(0) = nodep;
  pi(1) = 0;
  pi(2) = 0;
  pi(3) = nodep;
  pi(4) = 3;
  pi(5) = 3;
  pi(6) = nodep;
  pi(7) = 6;
  pi(8) = 6;
  pi(9) = nodep;

  matrix_type J;
  OpenTissue::math::optimization::detail::compute_jacobian( 
      A
    , OpenTissue::math::optimization::make_lower_mbd_bounds( pi, mu, lo )
    , OpenTissue::math::optimization::make_upper_mbd_bounds( pi, mu, hi )
    , bitmask        
    , J
    );

  double tol = 0.01;

  // upper      pi = inf
  BOOST_CHECK_CLOSE( double( J(0,0) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,6) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,7) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,8) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(0,9) ), double( 0.0 ), tol );

  // lower      pi = 0
  BOOST_CHECK_CLOSE( double( J(1,0) ), double( mu(1) ), tol );
  BOOST_CHECK_CLOSE( double( J(1,1) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,6) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,7) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,8) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(1,9) ), double( 0.0 ), tol );

  // active     pi = 0
  BOOST_CHECK_CLOSE( double( J(2,0) ), double( A(2,0) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,1) ), double( A(2,1) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,2) ), double( A(2,2) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,3) ), double( A(2,3) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,4) ), double( A(2,4) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,5) ), double( A(2,5) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,6) ), double( A(2,6) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,7) ), double( A(2,7) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,8) ), double( A(2,8) ), tol );
  BOOST_CHECK_CLOSE( double( J(2,9) ), double( A(2,9) ), tol );

  // lower      pi = inf
  BOOST_CHECK_CLOSE( double( J(3,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,3) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,6) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,7) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,8) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(3,9) ), double( 0.0 ), tol );

  // active     pi = 3
  BOOST_CHECK_CLOSE( double( J(4,0) ), double( A(4,0) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,1) ), double( A(4,1) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,2) ), double( A(4,2) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,3) ), double( A(4,3) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,4) ), double( A(4,4) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,5) ), double( A(4,5) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,6) ), double( A(4,6) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,7) ), double( A(4,7) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,8) ), double( A(4,8) ), tol );
  BOOST_CHECK_CLOSE( double( J(4,9) ), double( A(4,9) ), tol );

  // upper      pi = 3
  BOOST_CHECK_CLOSE( double( J(5,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,3) ), double( -mu(5) ), tol );
  BOOST_CHECK_CLOSE( double( J(5,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,5) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,6) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,7) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,8) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(5,9) ), double( 0.0 ), tol );

  // active     pi = inf
  BOOST_CHECK_CLOSE( double( J(6,0) ), double( A(6,0) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,1) ), double( A(6,1) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,2) ), double( A(6,2) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,3) ), double( A(6,3) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,4) ), double( A(6,4) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,5) ), double( A(6,5) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,6) ), double( A(6,6) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,7) ), double( A(6,7) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,8) ), double( A(6,8) ), tol );
  BOOST_CHECK_CLOSE( double( J(6,9) ), double( A(6,9) ), tol );

  // upper      pi = 6
  BOOST_CHECK_CLOSE( double( J(7,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,6) ), double( -mu(7) ), tol );
  BOOST_CHECK_CLOSE( double( J(7,7) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,8) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(7,9) ), double( 0.0 ), tol );

  // lower      pi = 6
  BOOST_CHECK_CLOSE( double( J(8,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,5) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,6) ), double( mu(8) ), tol );
  BOOST_CHECK_CLOSE( double( J(8,7) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,8) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( J(8,9) ), double( 0.0 ), tol );

  // active     pi = inf
  BOOST_CHECK_CLOSE( double( J(9,0) ), double( A(9,0) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,1) ), double( A(9,1) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,2) ), double( A(9,2) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,3) ), double( A(9,3) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,4) ), double( A(9,4) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,5) ), double( A(9,5) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,6) ), double( A(9,6) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,7) ), double( A(9,7) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,8) ), double( A(9,8) ), tol );
  BOOST_CHECK_CLOSE( double( J(9,9) ), double( A(9,9) ), tol );

}

BOOST_AUTO_TEST_SUITE_END();
