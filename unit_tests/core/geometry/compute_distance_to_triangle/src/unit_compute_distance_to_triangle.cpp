//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/geometry/geometry_compute_distance_to_triangle.h>
#include <cmath> // needed for std::sqrt

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_geometry_util_compute_distance_to_triangle);

  BOOST_AUTO_TEST_CASE(case_by_case_testing)
  {
    using std::sqrt;

    typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
    typedef math_types::vector3_type                         vector3_type;
    typedef math_types::real_type                            real_type;

    real_type tol = 0.0001;

    vector3_type p;
    vector3_type pi(-5,0,0);
    vector3_type pj(5,0,0);
    vector3_type pk(0,5,0);

    p = vector3_type(0,2.5,2);

    real_type d0 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d0, 2.0, tol);

    p = vector3_type(0,2.5,-2);
    real_type d1 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d1, 2.0, tol);

    p = vector3_type(0,7,0);
    real_type d2 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d2, 2.0, tol);

    p = vector3_type(-7,0,0);
    real_type d3 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d3, 2.0, tol);

    p = vector3_type(7,0,0);
    real_type d4 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d4, 2.0, tol);

    p = vector3_type(0,-2,0);
    real_type d5 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d5, 2.0, tol);

    p = vector3_type(5,5,0);
    real_type d6 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d6, 3.5355339059327376220042218105242, tol);

    p = vector3_type(-5,5,0);
    real_type d7 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d7, 3.5355339059327376220042218105242, tol);

    p = vector3_type(-5,-2,0);
    real_type d8 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d8, 2.0, tol);

    p = vector3_type(5,-2,0);
    real_type d9 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d9, 2.0, tol);

    p = vector3_type(6,1,0);
    real_type d10 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d10, sqrt(2.0), tol );

    p = vector3_type(-6,1,0);
    real_type d11 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d11, sqrt(2.0), tol );

    p = vector3_type(1,6,0);
    real_type d12 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d12, sqrt(2.0), tol );

    p = vector3_type(-1,6,0);
    real_type d13 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d13, sqrt(2.0), tol );

    p = vector3_type(0,0,1);
    real_type d14 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d14, 1.0, tol );

    p = vector3_type(5,0,1);
    real_type d15 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d15, 1.0, tol );

    p = vector3_type(-5,0,1);
    real_type d16 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d16, 1.0, tol );

    p = vector3_type(0,5,1);
    real_type d17 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d17, 1.0, tol );

    p = vector3_type(2.5,2.5,1);
    real_type d18 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d18, 1.0, tol );

    p = vector3_type(-2.5,2.5,1);
    real_type d19 = OpenTissue::geometry::compute_distance_to_triangle(p,pi,pj,pk);
    BOOST_CHECK_CLOSE(d19, 1.0, tol );

  }

BOOST_AUTO_TEST_SUITE_END();
