// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>
#include <OpenTissue/core/spline/spline.h>

#define BOOST_AUTO_TEST_MAIN
#include <boost/test/auto_unit_test.hpp>

// Boost Test declaration and Checking macros
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/test_tools.hpp>
#include <boost/test/floating_point_comparison.hpp>

typedef OpenTissue::spline::MathTypes<double, size_t>    math_types;
typedef math_types::vector_type                                vector_type;
typedef std::vector<double>                                    knot_container;
typedef std::vector<vector_type>                               point_container;

typedef OpenTissue::spline::NUBSpline<knot_container, point_container> spline_type;

BOOST_AUTO_TEST_SUITE(opentissue_spline_make_periodic);

BOOST_AUTO_TEST_CASE(test_make_periodic)
{
  knot_container U;

  U.push_back(0.0);
  U.push_back(0.0);
  U.push_back(0.0);  //k = 3
  U.push_back(1.0);
  U.push_back(2.0);
  U.push_back(3.0);
  U.push_back(4.0);  // n = 6  => |P| = 7
  U.push_back(5.0);
  U.push_back(5.0);
  U.push_back(5.0);  // m = 9  => |U| = 10

  point_container P;
  vector_type p0(2);  p0(0) = 0.0; p0(1) = 0.0;
  vector_type p1(2);  p1(0) = 1.0; p1(1) = 0.0;
  vector_type p2(2);  p2(0) = 2.0; p2(1) = 0.0;
  vector_type p3(2);  p3(0) = 3.0; p3(1) = 0.0;
  vector_type p4(2);  p4(0) = 4.0; p4(1) = 0.0;
  vector_type p5(2);  p5(0) = 5.0; p5(1) = 0.0;
  vector_type p6(2);  p6(0) = 6.0; p6(1) = 0.0;

  P.push_back(p0);
  P.push_back(p1);
  P.push_back(p2);
  P.push_back(p3);
  P.push_back(p4);
  P.push_back(p5);
  P.push_back(p6);

  spline_type spline(3,U,P);

  spline_type closed = OpenTissue::spline::make_periodic( spline );
}

BOOST_AUTO_TEST_SUITE_END();
