//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/geometry/geometry_barycentric.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_geometry_util_compute_distance_to_triangle);

BOOST_AUTO_TEST_CASE(inside_simplex_testing)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         V;
  typedef math_types::real_type                            T;

  V const a = V( 0.0, 0.0, 0.0);
  V const b = V(-1.0, 0.0, 0.0);
  V const c = V( 0.0, 1.0, 0.0);
  V const d = V( -2.0, -2.0, 1.0);

  // Test barycentric coordinates of a point lying on an edge
  {
    for(size_t i = 0;i<100;++i)
    {
      T WA = i*0.01;
      T WB = 1.0 - WA;
      V const p = WA*a + WB*b;
      T wa = 10.0;
      T wb = 10.0;
      OpenTissue::geometry::barycentric_geometric( a, b, p, wa, wb );
      BOOST_CHECK_CLOSE(wa, WA, 0.01 );
      BOOST_CHECK_CLOSE(wb, WB, 0.01 );
    }
  }
  // Test barycentric coordinates of a point lying on a triangle
  {
    for(size_t i = 1;i<10;++i)
    for(size_t j = 1;j<10;++j)
    for(size_t k = 1;k<10;++k)
    {
      T WA = i*0.1;
      T WB = j*0.1;
      T WC = k*0.1;
      T W = WA + WB + WC;
      WA /= W;
      WB /= W;
      WC /= W;
      V const p = WA*a + WB*b + WC*c;
      T wa = 10.0;
      T wb = 10.0;
      T wc = 10.0;
      // geometric method
      OpenTissue::geometry::barycentric_geometric( a, b, c, p, wa, wb, wc );
      BOOST_CHECK_CLOSE(wa, WA, 0.01 );
      BOOST_CHECK_CLOSE(wb, WB, 0.01 );
      BOOST_CHECK_CLOSE(wc, WC, 0.01 );

      wa = 10.0;
      wb = 10.0;
      wc = 10.0;
      // algebraic method
      OpenTissue::geometry::barycentric_algebraic( a, b, c, p, wa, wb, wc );
      BOOST_CHECK_CLOSE(wa, WA, 0.01 );
      BOOST_CHECK_CLOSE(wb, WB, 0.01 );
      BOOST_CHECK_CLOSE(wc, WC, 0.01 );

    }
  }
  // Test barycentric coordinates of a point lying in a tetrahedron
  {
    for(size_t i = 1;i<10;++i)
    for(size_t j = 1;j<10;++j)
    for(size_t k = 1;k<10;++k)
    for(size_t m = 1;m<10;++m)
    {
      T WA = i*0.1;
      T WB = j*0.1;
      T WC = k*0.1;
      T WD = m*0.1;
      T W = WA + WB + WC + WD;
      WA /= W;
      WB /= W;
      WC /= W;
      WD /= W;
      V const p = WA*a + WB*b + WC*c+ WD*d;
      T wa = 10.0;
      T wb = 10.0;
      T wc = 10.0;
      T wd = 10.0;
      // Geometric method
      OpenTissue::geometry::barycentric_geometric( a, b, c, d, p, wa, wb, wc, wd );
      BOOST_CHECK_CLOSE(wa, WA, 0.01 );
      BOOST_CHECK_CLOSE(wb, WB, 0.01 );
      BOOST_CHECK_CLOSE(wc, WC, 0.01 );
      BOOST_CHECK_CLOSE(wd, WD, 0.01 );

      wa = 10.0;
      wb = 10.0;
      wc = 10.0;
      wd = 10.0;
      // algebraic method
      OpenTissue::geometry::barycentric_algebraic( a, b, c, d, p, wa, wb, wc, wd );
      BOOST_CHECK_CLOSE(wa, WA, 0.01 );
      BOOST_CHECK_CLOSE(wb, WB, 0.01 );
      BOOST_CHECK_CLOSE(wc, WC, 0.01 );
      BOOST_CHECK_CLOSE(wd, WD, 0.01 );
    }
  }


}

BOOST_AUTO_TEST_SUITE_END();
