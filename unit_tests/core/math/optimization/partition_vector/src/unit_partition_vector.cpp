//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <OpenTissue/core/math/optimization/optimization_compute_index_reordering.h>
#include <OpenTissue/core/math/optimization/optimization_partition_vector.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_partition_vector);

BOOST_AUTO_TEST_CASE(test_case)
{
  typedef ublas::vector<size_t>   idx_vector_type;

  idx_vector_type bitmask;

  bitmask.resize(10,false);

  bitmask(2) = OpenTissue::math::optimization::IN_ACTIVE;
  bitmask(4) = OpenTissue::math::optimization::IN_ACTIVE;
  bitmask(6) = OpenTissue::math::optimization::IN_ACTIVE;
  bitmask(9) = OpenTissue::math::optimization::IN_ACTIVE;
  bitmask(1) = OpenTissue::math::optimization::IN_LOWER;
  bitmask(3) = OpenTissue::math::optimization::IN_LOWER;
  bitmask(8) = OpenTissue::math::optimization::IN_LOWER;
  bitmask(0) = OpenTissue::math::optimization::IN_UPPER;
  bitmask(5) = OpenTissue::math::optimization::IN_UPPER;
  bitmask(7) = OpenTissue::math::optimization::IN_UPPER;

  idx_vector_type old2new;
  idx_vector_type new2old;

  OpenTissue::math::optimization::compute_index_reordering( bitmask, old2new, new2old );

  ublas::vector<double> rhs;
  rhs.resize(10,false);

  rhs(0) = 1.0;
  rhs(1) = 2.0;
  rhs(2) = 3.0;
  rhs(3) = 4.0;
  rhs(4) = 5.0;
  rhs(5) = 6.0;
  rhs(6) = 7.0;
  rhs(7) = 8.0;
  rhs(8) = 9.0;
  rhs(9) = 10.0;

  ublas::vector<double> rhs_a;
  ublas::vector<double> rhs_b;
  OpenTissue::math::optimization::partition_vector( rhs, bitmask, old2new, 4, 6, rhs_a, rhs_b );

  BOOST_CHECK( rhs_a.size() == 4);
  BOOST_CHECK( rhs_b.size() == 6);

  double tol =0.01;
  BOOST_CHECK_CLOSE( double( rhs_a( 0 ) ), double( rhs( new2old(0) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_a( 1 ) ), double( rhs( new2old(1) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_a( 2 ) ), double( rhs( new2old(2) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_a( 3 ) ), double( rhs( new2old(3) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_b( 0 ) ), double( rhs( new2old(0+4) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_b( 1 ) ), double( rhs( new2old(1+4) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_b( 2 ) ), double( rhs( new2old(2+4) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_b( 3 ) ), double( rhs( new2old(3+4) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_b( 4 ) ), double( rhs( new2old(4+4) ) ), tol );
  BOOST_CHECK_CLOSE( double( rhs_b( 5 ) ), double( rhs( new2old(5+4) ) ), tol );
}

BOOST_AUTO_TEST_SUITE_END();
