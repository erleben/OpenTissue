//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/continuous/continuous_default_motion_policy.h>
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

BOOST_AUTO_TEST_SUITE(opentissue_collision_continuous_default_motion_policy);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         V;
  typedef math_types::real_type                            T;
  typedef math_types::quaternion_type                      Q;
  typedef math_types::coordsys_type                        X;

  X from;
  X to;

  from.identity();
  to.T() = V(1.0,2.0,3.0);
  to.Q().Ru( 3.0, V(1.0,0.0,0.0) );
  V v;
  V w;
  OpenTissue::collision::continuous::DefaultMotionPolicy::compute_velocities( from, to, 1.0, v, w);

  BOOST_CHECK_CLOSE( v(0), to.T()(0), 0.01 );
  BOOST_CHECK_CLOSE( v(1), to.T()(1), 0.01 );
  BOOST_CHECK_CLOSE( v(2), to.T()(2), 0.01 );
  BOOST_CHECK_CLOSE( w(0), 3.0, 0.01 );
  BOOST_CHECK_CLOSE( w(1), 0.0, 0.01 );
  BOOST_CHECK_CLOSE( w(2), 0.0, 0.01 );


  T tau = 0.5;
  X cur = OpenTissue::collision::continuous::DefaultMotionPolicy::integrate_motion(from, tau, v, w);

  BOOST_CHECK_CLOSE( 0.5, cur.T()(0), 0.01 );
  BOOST_CHECK_CLOSE( 1.0, cur.T()(1), 0.01 );
  BOOST_CHECK_CLOSE( 1.5, cur.T()(2), 0.01 );

  T theta;
  V n;
  OpenTissue::math::get_axis_angle( cur.Q(), n, theta );

  BOOST_CHECK_CLOSE( theta, 1.5, 0.01 );
  BOOST_CHECK_CLOSE( n(0), 1.0, 0.01 );
  BOOST_CHECK_CLOSE( n(1), 0.0, 0.01 );
  BOOST_CHECK_CLOSE( n(2), 0.0, 0.01 );


  OpenTissue::gjk::Sphere<math_types> const A;
  OpenTissue::gjk::Sphere<math_types> const B;
  X X_a;
  X X_b;

  X_a.T() = V(-2.0,0.0,0.0);
  X_a.Q().identity();

  X_b.T() = V( 2.0,0.0,0.0);
  X_b.Q().identity();

  V p_a;
  V p_b;

  OpenTissue::collision::continuous::DefaultMotionPolicy::compute_closest_points( X_a, A, X_b, B, p_a, p_b);

  BOOST_CHECK_CLOSE( p_a(0), -1.0, 0.1);
  BOOST_CHECK_CLOSE( p_a(1),  0.0, 0.1);
  BOOST_CHECK_CLOSE( p_a(2),  0.0, 0.1);

  BOOST_CHECK_CLOSE( p_b(0),  1.0, 0.1);
  BOOST_CHECK_CLOSE( p_b(1),  0.0, 0.1);
  BOOST_CHECK_CLOSE( p_b(2),  0.0, 0.1);

}


BOOST_AUTO_TEST_SUITE_END();
