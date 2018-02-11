//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_compute_index_reordering.h>
#include <OpenTissue/core/math/optimization/non_smooth_newton/optimization_compute_partitioned_jacobian.h>
#include <OpenTissue/core/math/optimization/optimization_make_mbd_bounds.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_non_smooth_newton_compute_partitioned_jacobian);

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

  matrix_type A_aa;
  matrix_type A_ab;
  matrix_type C;
  matrix_type D;

  OpenTissue::math::optimization::detail::compute_partitioned_jacobian( 
    A
    , OpenTissue::math::optimization::make_lower_mbd_bounds( pi, mu, lo )
    , OpenTissue::math::optimization::make_upper_mbd_bounds( pi, mu, hi )
    ,  bitmask
    , old2new
    ,  4
    ,  6
    ,  A_aa
    ,  A_ab
    ,  C
    ,  D
    );

  double tol = 0.01;

  // old : new   set      old : new
  //  2 <-> 0    active
  //  4 <-> 1    active
  //  6 <-> 2    active
  //  9 <-> 3    active
  //  1 <-> 4    lower      0 -> 7         (4,4) (4,7)     ->   D(0,0) D(0,3)
  //  3 <-> 5    lower      -              (5,5)           ->   D(1,1)
  //  8 <-> 6    lower      6 -> 2         (6,6) (6,2)     ->   D(2,2) C(2,2)
  //  0 <-> 7    upper      -              (7,7)           ->   D(3,3)
  //  5 <-> 8    upper      3 -> 5         (8,8) (8,5)     ->   D(4,4) D(4,1)
  //  7 <-> 9    upper      6 -> 2         (9,9) (9,2)     ->   D(5,5) C(5,2)

  BOOST_CHECK( A_aa.size1()==4 );
  BOOST_CHECK( A_aa.size2()==4 );
  for(size_type i = 0;i<4;++i)
  {
    for(size_type j = 0;j<4;++j)
    {
      BOOST_CHECK_CLOSE( double( A_aa(i,j) ), double( A(new2old(i),new2old(j)) ), tol );
    }
  }

  BOOST_CHECK( A_ab.size1()==4 );
  BOOST_CHECK( A_ab.size2()==6 );
  for(size_type i = 0;i<4;++i)
  {
    for(size_type j = 0;j<6;++j)
    {
      BOOST_CHECK_CLOSE( double( A_ab(i,j) ), double( A(new2old(i),new2old(j+4)) ), tol );
    }
  }

  BOOST_CHECK( C.size1()==6 );
  BOOST_CHECK( C.size2()==4 );
  BOOST_CHECK_CLOSE( double( C(0,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(0,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(0,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(0,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(1,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(1,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(1,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(1,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(2,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(2,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(2,2) ), double( mu(8) ), tol );
  BOOST_CHECK_CLOSE( double( C(2,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(4,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(4,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(4,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(4,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(5,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(5,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( C(5,2) ), double( -mu(7) ), tol );
  BOOST_CHECK_CLOSE( double( C(5,3) ), double( 0.0 ), tol );

  BOOST_CHECK( D.size1()==6 );
  BOOST_CHECK( D.size2()==6 );

  BOOST_CHECK_CLOSE( double( D(0,0) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(0,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(0,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(0,3) ), double( mu(1) ), tol );
  BOOST_CHECK_CLOSE( double( D(0,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(0,5) ), double( 0.0 ), tol );

  BOOST_CHECK_CLOSE( double( D(1,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(1,1) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(1,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(1,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(1,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(1,5) ), double( 0.0 ), tol );

  BOOST_CHECK_CLOSE( double( D(2,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(2,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(2,2) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(2,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(2,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(2,5) ), double( 0.0 ), tol );

  BOOST_CHECK_CLOSE( double( D(3,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(3,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(3,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(3,3) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(3,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(3,5) ), double( 0.0 ), tol );

  BOOST_CHECK_CLOSE( double( D(4,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(4,1) ), double( -mu(5) ), tol );
  BOOST_CHECK_CLOSE( double( D(4,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(4,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(4,4) ), double( 1.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(4,5) ), double( 0.0 ), tol );

  BOOST_CHECK_CLOSE( double( D(5,0) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(5,1) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(5,2) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(5,3) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(5,4) ), double( 0.0 ), tol );
  BOOST_CHECK_CLOSE( double( D(5,5) ), double( 1.0 ), tol );

}

BOOST_AUTO_TEST_SUITE_END();
