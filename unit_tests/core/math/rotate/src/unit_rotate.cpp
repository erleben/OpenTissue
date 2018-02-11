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

BOOST_AUTO_TEST_SUITE(opentissue_math_quaternion_rotate);

BOOST_AUTO_TEST_CASE(simple_test)
{
  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;

  typedef math_types::value_traits     value_traits;
  typedef math_types::real_type        T;
  typedef math_types::vector3_type     V;
  typedef math_types::quaternion_type  Q;

  {
    // Set up a test rotation
    Q q;
    T const phi = value_traits::pi_half();
    V const m = V( 1.0, 0.0, 0.0 );
    q.Ru( phi, m);
    V const n = V( 0.0, 1.0, 0.0 );
    // Try to rotate n-vector
    V const k = rotate( q, n);
    // Test if result is what we expect
    BOOST_CHECK( fabs( k(0) ) < 10e-10 );
    BOOST_CHECK( fabs( k(1) ) < 10e-10 );
    BOOST_CHECK( fabs( 1.0 - k(2) ) < 10e-10 );
  }
  {
    // Set up a test rotation
    Q q;
    T const phi = value_traits::pi_half();
    V const m = V( 1.0, 0.0, 0.0 );
    q.Ru( phi, m);
    V const n = V( 0.0, 0.0, 1.0 );
    // Try to rotate n-vector
    V const k = rotate( q, n);
    // Test if result is what we expect
    BOOST_CHECK( fabs( k(0) ) < 10e-10 );
    BOOST_CHECK( fabs( 1.0 + k(1) ) < 10e-10 );
    BOOST_CHECK( fabs( k(2) ) < 10e-10 );
  }
  {
    // Set up a test rotation
    Q q;
    T const phi = value_traits::pi_half();
    V const m = V( 1.0, 0.0, 0.0 );
    q.Ru( phi, m);
    V const n = V( 0.0, -1.0, 0.0 );
    // Try to rotate n-vector
    V const k = rotate( q, n);
    // Test if result is what we expect
    BOOST_CHECK( fabs( k(0) ) < 10e-10 );
    BOOST_CHECK( fabs( k(1) ) < 10e-10 );
    BOOST_CHECK( fabs( 1.0 + k(2) ) < 10e-10 );
  }
  {
    // Set up a test rotation
    Q q;
    T const phi = value_traits::pi_half();
    V const m = V( 1.0, 0.0, 0.0 );
    q.Ru( phi, m);
    V const n = V( 0.0, 0.0, -1.0 );
    // Try to rotate n-vector
    V const k = rotate( q, n);
    // Test if result is what we expect
    BOOST_CHECK( fabs( k(0) ) < 10e-10 );
    BOOST_CHECK( fabs( 1.0 - k(1) ) < 10e-10 );
    BOOST_CHECK( fabs( k(2) ) < 10e-10 );
  }
  {
    // Set up a test rotation
    Q q;
    T const phi = value_traits::pi_half();
    V const m = V( 0.0, 1.0, 0.0 );
    q.Ru( phi, m);
    V const n = V( 1.0, 0.0, 0.0 );
    // Try to rotate n-vector
    V const k = rotate( q, n);
    // Test if result is what we expect
    BOOST_CHECK( fabs( k(0) ) < 10e-10 );
    BOOST_CHECK( fabs( k(1) ) < 10e-10 );
    BOOST_CHECK( fabs( 1.0 + k(2) ) < 10e-10 );
  }
  {
    // Set up a test rotation
    Q q;
    T const phi = value_traits::pi_half();
    V const m = V( 0.0, 0.0, 1.0 );
    q.Ru( phi, m);
    V const n = V( 1.0, 0.0, 0.0 );
    // Try to rotate n-vector
    V const k = rotate( q, n);
    // Test if result is what we expect
    BOOST_CHECK( fabs( k(0) ) < 10e-10 );
    BOOST_CHECK( fabs( 1.0 - k(1) ) < 10e-10 );
    BOOST_CHECK( fabs( k(2) ) < 10e-10 );
  }
}

BOOST_AUTO_TEST_SUITE_END();
