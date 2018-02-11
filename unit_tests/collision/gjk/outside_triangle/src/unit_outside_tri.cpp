//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_outside_triangle.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_outside_triangle);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         V;

  V const a = V(0.0, 0.0, 0.0);
  V const b = V(1.0, 0.0, 0.0);;
  V const c = V(0.0, 1.0, 0.0);;
  V const q = V(0.33, 0.33, -1.0);

  {
    V p = V(0.33, 0.33,  1.0);
    bool outside = OpenTissue::gjk::detail::outside_triangle(p, a, b, c, q );
    BOOST_CHECK( outside );
  }
  {
    V p = V(0.1, 0.1,  -1.0);
    bool outside = OpenTissue::gjk::detail::outside_triangle(p, a, b, c, q );
    BOOST_CHECK( !outside );
  }
  {
    V p = V(0.1, 0.1,  0.0);
    bool outside = OpenTissue::gjk::detail::outside_triangle(p, a, b, c, q );
    BOOST_CHECK( outside );
  }
}

BOOST_AUTO_TEST_SUITE_END();
