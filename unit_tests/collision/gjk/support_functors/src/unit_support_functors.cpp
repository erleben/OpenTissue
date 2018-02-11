//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_support_functors.h>



#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_support_functions);


BOOST_AUTO_TEST_CASE(case_by_case_testing)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;

  typedef math_types::vector3_type                         V;

  OpenTissue::gjk::Box<math_types>       const box;
  OpenTissue::gjk::Capsule<math_types>   const capsule;
  OpenTissue::gjk::Cone<math_types>      const cone;
  OpenTissue::gjk::Cylinder<math_types>  const cylinder;
  OpenTissue::gjk::Ellipsoid<math_types> const ellipsoid;
  OpenTissue::gjk::Sphere<math_types>    const sphere;
  OpenTissue::gjk::Point<math_types>           point;


  V const zero     = V( 0.0, 0.0, 0.0);
  V const up       = V( 0.0, 0.0, 1.0);
  V const down     = V( 0.0, 0.0,-1.0);
  V const right    = V( 1.0, 0.0, 0.0);
  V const left     = V(-1.0, 0.0, 0.0);
  V const forward  = V( 0.0, 1.0, 0.0);
  V const backward = V( 0.0,-1.0, 0.0);


  // Test point type
  {
    point.point()(0) = 1.0;
    point.point()(1) = 2.0;
    point.point()(2) = 3.0;

    V const o = point( zero );
    BOOST_CHECK( o == point.point() );
    V const p = point( up );
    BOOST_CHECK( p == point.point() );
    V const q = point( right );
    BOOST_CHECK( q == point.point() );
    V const r = point( down );
    BOOST_CHECK( r == point.point() );
    V const s = point( forward );
    BOOST_CHECK( s == point.point() );
  }

  // Test what happens if zero vector is given as search direction
  {
    V const s = box( zero );
    BOOST_CHECK( zero != s );
  }
  {
    V const s = capsule( zero );
    BOOST_CHECK( zero != s );
  }
  {
    V const s = cone( zero );
    BOOST_CHECK( zero != s );
  }
  {
    V const s = cylinder( zero );
    BOOST_CHECK( zero != s );
  }
  {
    V const s = ellipsoid( zero );
    BOOST_CHECK( zero != s );
  }
  {
    V const s = sphere( zero );
    BOOST_CHECK( zero != s );
  }
  // Test what happen if up vector is given
  {
    V const s = box( up );

    BOOST_CHECK( s(0) <= box.half_extent()(0) );
    BOOST_CHECK( s(1) <= box.half_extent()(1) );
    BOOST_CHECK( s(0) >= -box.half_extent()(0) );
    BOOST_CHECK( s(1) >= -box.half_extent()(1) );
    BOOST_CHECK( s(2) == box.half_extent()(2) );
  }
  {
    V const s = capsule( up );
    BOOST_CHECK( s == V(0,0,2) );
  }
  {
    V const s = cone( up );
    BOOST_CHECK( s == up);
  }
  {
    V const s = cylinder( up );
    BOOST_CHECK( s == up );
  }
  {
    V const s = ellipsoid( up );
    BOOST_CHECK( s == up );
  }
  {
    V const s = sphere( up );
    BOOST_CHECK( s == up);
  }

  // Test what happen if right vector is given
  {
    V const s = box( right );

    BOOST_CHECK( s(0) == box.half_extent()(0) );
    BOOST_CHECK( s(1) <= box.half_extent()(1) );
    BOOST_CHECK( s(2) <= box.half_extent()(2) );
    BOOST_CHECK( s(1) >= -box.half_extent()(1) );
    BOOST_CHECK( s(2) >= -box.half_extent()(2) );
  }
  {
    V const s = capsule( right );
    BOOST_CHECK( s == right );
  }
  {
    V const s = cone( right );
    BOOST_CHECK( s == V(1,0,-1));
  }
  {
    V const s = cylinder( right );
    BOOST_CHECK( s(0) == cylinder.radius() );
    BOOST_CHECK( s(1) == 0.0 );
    BOOST_CHECK( s(2) <=  cylinder.half_height() );
    BOOST_CHECK( s(2) >= -cylinder.half_height() );
  }
  {
    V const s = ellipsoid( right );
    BOOST_CHECK( s == right );
  }
  {
    V const s = sphere( right );
    BOOST_CHECK( s == right);
  }



  // Test what happen if down vector is given
  {
    V const s = box( down );

    BOOST_CHECK( s(1) == -box.half_extent()(0) );

    BOOST_CHECK( s(0) <= box.half_extent()(0) );
    BOOST_CHECK( s(1) <= box.half_extent()(1) );
    BOOST_CHECK( s(0) >= -box.half_extent()(0) );
    BOOST_CHECK( s(1) >= -box.half_extent()(1) );
  }
  {
    V const s = capsule( down );
    BOOST_CHECK( s == V(0,0,-2) );
  }
  {
    V const s = cone( down );
    BOOST_CHECK( s == down );
  }
  {
    V const s = cylinder( down );
    BOOST_CHECK( s(0) == 0.0 );
    BOOST_CHECK( s(1) == 0.0 );
    BOOST_CHECK( s(2) == -cylinder.half_height() );
  }
  {
    V const s = ellipsoid( down );
    BOOST_CHECK( s == down );
  }
  {
    V const s = sphere( down );
    BOOST_CHECK( s == down);
  }


}

BOOST_AUTO_TEST_SUITE_END();
