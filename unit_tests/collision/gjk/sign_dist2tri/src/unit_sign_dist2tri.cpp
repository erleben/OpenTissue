//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/collision/gjk/gjk_signed_distance_to_triangle.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <cmath>

using namespace OpenTissue;

BOOST_AUTO_TEST_SUITE(opentissue_collision_gjk_signed_distance_to_triangle);

BOOST_AUTO_TEST_CASE(case_by_case_test)
{
  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;


  vector3_type a = vector3_type(0.0, 0.0, 0.0);
  vector3_type b = vector3_type(1.0, 0.0, 0.0);;
  vector3_type c = vector3_type(0.0, 1.0, 0.0);;
  vector3_type q = vector3_type(0.33, 0.33, -1.0);

  {
    vector3_type p = vector3_type(0.33, 0.33,  1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_triangle(p, a, b, c, q );
    BOOST_CHECK_CLOSE( sign_p, 1.0, 0.01 );
  }

  {
    vector3_type p = vector3_type(0.1, 0.1,  -1.0);
    real_type sign_p = 0.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_triangle(p, a, b, c, q );
    BOOST_CHECK_CLOSE( sign_p, -1.0, 0.01 );
  }

  {
    vector3_type p = vector3_type(0.1, 0.1,  0.0);
    real_type sign_p = 10.0; 
    sign_p = OpenTissue::gjk::detail::signed_distance_to_triangle(p, a, b, c, q );
    BOOST_CHECK_CLOSE( sign_p, 0.0, 0.01 );
  }

}

BOOST_AUTO_TEST_SUITE_END();
