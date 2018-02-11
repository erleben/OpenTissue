//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>


#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_axis_angle);

BOOST_AUTO_TEST_CASE(simple_test)
{

  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;

  typedef math_types::real_type        T;
  typedef math_types::vector3_type     V;
  typedef math_types::quaternion_type  Q;

  // Small positive angle
  {
    Q q;
    T const phi = 0.01;
    V const m = unit( V( 1.0, 1.0, 1.0 ) );
    q.Ru( phi, m);

    T theta;
    V n;
    OpenTissue::math::get_axis_angle( q, n, theta );

    BOOST_CHECK_CLOSE( phi, theta, 0.01 );
    BOOST_CHECK_CLOSE( m(0), n(0), 0.01 );
    BOOST_CHECK_CLOSE( m(1), n(1), 0.01 );
    BOOST_CHECK_CLOSE( m(2), n(2), 0.01 );
  }
  // See what happens if axis is flipped
  {
    Q q;
    T const phi = 0.01;
    V const m = -unit( V( 1.0, 1.0, 1.0 ) );
    q.Ru( phi, m);

    T theta;
    V n;
    OpenTissue::math::get_axis_angle( q, n, theta );

    BOOST_CHECK_CLOSE( phi, theta, 0.01 );
    BOOST_CHECK_CLOSE( m(0), n(0), 0.01 );
    BOOST_CHECK_CLOSE( m(1), n(1), 0.01 );
    BOOST_CHECK_CLOSE( m(2), n(2), 0.01 );
  }
  // Pick larger angle
  {
    Q q;
    T const phi = 3.0;
    V const m = unit( V( 1.0, 1.0, 1.0 ) );
    q.Ru( phi, m);

    T theta;
    V n;
    OpenTissue::math::get_axis_angle( q, n, theta );

    BOOST_CHECK_CLOSE( phi, theta, 0.01 );
    BOOST_CHECK_CLOSE( m(0), n(0), 0.01 );
    BOOST_CHECK_CLOSE( m(1), n(1), 0.01 );
    BOOST_CHECK_CLOSE( m(2), n(2), 0.01 );
  }
  // Pick larger negative angle, the positive angle version representation should be returned!
  {
    Q q;
    T const phi = -3.0;
    V const m = unit( V( 1.0, 1.0, 1.0 ) );
    q.Ru( phi, m);

    T theta;
    V n;
    OpenTissue::math::get_axis_angle( q, n, theta );

    BOOST_CHECK_CLOSE( -phi, theta, 0.01 );
    BOOST_CHECK_CLOSE( -m(0), n(0), 0.01 );
    BOOST_CHECK_CLOSE( -m(1), n(1), 0.01 );
    BOOST_CHECK_CLOSE( -m(2), n(2), 0.01 );
  }
  // Flip the axis
  {
    Q q;
    T const phi = -3.0;
    V const m = -unit( V( 1.0, 1.0, 1.0 ) );
    q.Ru( phi, m);

    T theta;
    V n;
    OpenTissue::math::get_axis_angle( q, n, theta );

    BOOST_CHECK_CLOSE( -phi, theta, 0.01 );
    BOOST_CHECK_CLOSE( -m(0), n(0), 0.01 );
    BOOST_CHECK_CLOSE( -m(1), n(1), 0.01 );
    BOOST_CHECK_CLOSE( -m(2), n(2), 0.01 );
  }
  // Large positive angle
  {
    Q q;
    T const phi = 6.0;
    V const m = unit( V( 1.0, 1.0, 1.0 ) );
    q.Ru( phi, m);

    T theta;
    V n;
    OpenTissue::math::get_axis_angle( q, n, theta );

    BOOST_CHECK_CLOSE( phi, theta, 0.01 );
    BOOST_CHECK_CLOSE( m(0), n(0), 0.01 );
    BOOST_CHECK_CLOSE( m(1), n(1), 0.01 );
    BOOST_CHECK_CLOSE( m(2), n(2), 0.01 );
  }

}

BOOST_AUTO_TEST_SUITE_END();
