//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_compute_index_reordering.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_compute_index_reordering);

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

  BOOST_CHECK( old2new( 2 ) == 0 );
  BOOST_CHECK( old2new( 4 ) == 1 );
  BOOST_CHECK( old2new( 6 ) == 2 );
  BOOST_CHECK( old2new( 9 ) == 3 );
  BOOST_CHECK( old2new( 1 ) == 4 );
  BOOST_CHECK( old2new( 3 ) == 5 );
  BOOST_CHECK( old2new( 8 ) == 6 );
  BOOST_CHECK( old2new( 0 ) == 7 );
  BOOST_CHECK( old2new( 5 ) == 8 );
  BOOST_CHECK( old2new( 7 ) == 9 );
  BOOST_CHECK( new2old( 0 ) == 2 );
  BOOST_CHECK( new2old( 1 ) == 4 );
  BOOST_CHECK( new2old( 2 ) == 6 );
  BOOST_CHECK( new2old( 3 ) == 9 );
  BOOST_CHECK( new2old( 4 ) == 1 );
  BOOST_CHECK( new2old( 5 ) == 3 );
  BOOST_CHECK( new2old( 6 ) == 8 );
  BOOST_CHECK( new2old( 7 ) == 0 );
  BOOST_CHECK( new2old( 8 ) == 5 );
  BOOST_CHECK( new2old( 9 ) == 7 );

}

BOOST_AUTO_TEST_SUITE_END();
