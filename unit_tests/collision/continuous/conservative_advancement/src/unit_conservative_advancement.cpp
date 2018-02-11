//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/continuous/continuous_default_motion_policy.h>
#include <OpenTissue/collision/continuous/continuous_conservative_advancement.h>
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

BOOST_AUTO_TEST_SUITE(opentissue_collision_continuous_conservative_advancement);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::value_traits                         value_traits;
  typedef math_types::vector3_type                         V;
  typedef math_types::real_type                            T;
  typedef math_types::quaternion_type                      Q;
  typedef math_types::coordsys_type                        X;

  // Setup a central impact, two sphere hitting in perfect symmetry, totally independent of their rotationnal motion!
  {
    OpenTissue::gjk::Sphere<math_types> const A;
    OpenTissue::gjk::Sphere<math_types> const B;

    X T_a;
    X T_b;

    T_a.T() = V(-2.0, 0.0, 0.0);
    T_a.Q().identity();
    V v_a = V( 2.0, 0.0, 0.0);
    V w_a = V( 0.0, 5.0, 0.0);
    T r_max_a = A.radius();

    T_b.T() = V( 2.0, 0.0, 0.0);
    T_b.Q().identity();
    V v_b = V(-2.0, 0.0, 0.0);
    V w_b = V( 0.0, 0.0, 5.0);
    T r_max_b = B.radius();

    OpenTissue::collision::continuous::DefaultMotionPolicy const motion_policy = OpenTissue::collision::continuous::DefaultMotionPolicy();
    size_t const max_iterations = 100u;
    T      const epsilon = 0.0001;
    T toi;
    size_t iterations;
    V p_a;
    V p_b;

    bool impact = OpenTissue::collision::continuous::conservative_advancement(
      T_a
      , v_a
      , w_a
      , A
      , r_max_a
      , T_b
      , v_b
      , w_b
      , B
      , r_max_b
      , p_a
      , p_b
      , toi
      , iterations
      , epsilon
      , value_traits::one()
      , max_iterations
      , motion_policy 
      );

    BOOST_CHECK( impact );
    BOOST_CHECK_CLOSE( toi, 0.5, 0.01 );

    BOOST_CHECK( fabs( p_a(0) ) < epsilon );
    BOOST_CHECK( fabs( p_a(1) ) < epsilon );
    BOOST_CHECK( fabs( p_a(2) ) < epsilon );
    BOOST_CHECK( fabs( p_b(0) ) < epsilon );
    BOOST_CHECK( fabs( p_b(1) ) < epsilon );
    BOOST_CHECK( fabs( p_b(2) ) < epsilon );
  }

  // Setup separating motion, two spheres moving away from each other
  {
    OpenTissue::gjk::Sphere<math_types> const A;
    OpenTissue::gjk::Sphere<math_types> const B;

    X T_a;
    X T_b;

    T_a.T() = V(-2.0, 0.0, 0.0);
    T_a.Q().identity();
    V v_a = V( -2.0, 0.0, 0.0);
    V w_a = V( 0.0, 0.0, 0.0);
    T r_max_a = A.radius();

    T_b.T() = V( 2.0, 0.0, 0.0);
    T_b.Q().identity();
    V v_b = V( 2.0, 0.0, 0.0);
    V w_b = V( 0.0, 0.0, 0.0);
    T r_max_b = B.radius();


    OpenTissue::collision::continuous::DefaultMotionPolicy const motion_policy = OpenTissue::collision::continuous::DefaultMotionPolicy();
    size_t const max_iterations = 100u;
    T      const epsilon = 0.0001;
    T toi;
    size_t iterations;
    V p_a;
    V p_b;

    bool impact = OpenTissue::collision::continuous::conservative_advancement(
      T_a
      , v_a
      , w_a
      , A
      , r_max_a
      , T_b
      , v_b
      , w_b
      , B
      , r_max_b
      , p_a
      , p_b
      , toi
      , iterations
      , epsilon
      , value_traits::one()
      , max_iterations
      , motion_policy 
      );

    BOOST_CHECK( !impact );
  }

  // Setup two sphere moving close by each other but never impacting
  {
    OpenTissue::gjk::Sphere<math_types> const A;
    OpenTissue::gjk::Sphere<math_types> const B;

    X T_a;
    X T_b;

    T_a.T() = V(-2.0, 1.01, 0.0);
    T_a.Q().identity();
    V v_a = V( -2.0, 0.0, 0.0);
    V w_a = V( 0.0, 5.0, 0.0);
    T r_max_a = A.radius();

    T_b.T() = V( 2.0, -1.01, 0.0);
    T_b.Q().identity();
    V v_b = V( 2.0, 0.0, 0.0);
    V w_b = V( 0.0, 0.0, 5.0);
    T r_max_b = B.radius();

    OpenTissue::collision::continuous::DefaultMotionPolicy const motion_policy = OpenTissue::collision::continuous::DefaultMotionPolicy();
    size_t const max_iterations = 100u;
    T      const epsilon = 0.0001;
    T toi;
    size_t iterations;
    V p_a;
    V p_b;

    bool impact = OpenTissue::collision::continuous::conservative_advancement(
      T_a
      , v_a
      , w_a
      , A
      , r_max_a
      , T_b
      , v_b
      , w_b
      , B
      , r_max_b
      , p_a
      , p_b
      , toi
      , iterations
      , epsilon
      , value_traits::one()
      , max_iterations
      , motion_policy 
      );

    BOOST_CHECK( !impact );
  }

  // Setup test case where only rotational motion is of importance!
  {
    OpenTissue::gjk::Cylinder<math_types> A;
    OpenTissue::gjk::Sphere<math_types> B;

    A.half_height() = 10.0;
    A.radius()      =  1.0;
    B.radius()      =  1.0;

    X T_a;
    X T_b;

    T_a.T() = V(0.0, 0.0, 2.0);
    T_a.Q().identity();

    V v_a = V( 0.0, 0.0, 0.0);
    V w_a = V( -value_traits::pi(), 0.0, 0.0 );    
    T r_max_a = sqrt( A.half_height()*A.half_height() + A.radius()*A.radius() );

    T_b.T() = V( 0.0, 10.0, 0.0);
    T_b.Q().identity();

    V v_b = V( 0.0, 0.0, 0.0);
    V w_b = V( 0.0, 0.0, 0.0);
    T r_max_b = B.radius();

    OpenTissue::collision::continuous::DefaultMotionPolicy const motion_policy = OpenTissue::collision::continuous::DefaultMotionPolicy();
    size_t const max_iterations = 100u;
    T      const epsilon = 0.0001;
    T toi;
    size_t iterations;
    V p_a;
    V p_b;

    bool impact = OpenTissue::collision::continuous::conservative_advancement(
      T_a
      , v_a
      , w_a
      , A
      , r_max_a
      , T_b
      , v_b
      , w_b
      , B
      , r_max_b
      , p_a
      , p_b
      , toi
      , iterations
      , epsilon
      , value_traits::one()
      , max_iterations
      , motion_policy 
      );

    BOOST_CHECK( impact );
    BOOST_CHECK_CLOSE( toi, 0.5, 0.01 );
    BOOST_CHECK( norm(p_a-p_b) < epsilon );
  }


}


BOOST_AUTO_TEST_SUITE_END();
