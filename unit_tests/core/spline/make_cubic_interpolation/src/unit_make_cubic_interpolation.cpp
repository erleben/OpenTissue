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

BOOST_AUTO_TEST_SUITE(opentissue_spline_make_cubic_interpolation);

BOOST_AUTO_TEST_CASE(test_make_cubic_interpolation)
{
  point_container X;

  vector_type p0(2);  p0(0) = 0.0; p0(1) = 0.0;
  vector_type p1(2);  p1(0) = 1.0; p1(1) = 0.0;
  vector_type p2(2);  p2(0) = 2.0; p2(1) = 0.0;
  vector_type p3(2);  p3(0) = 3.0; p3(1) = 0.0;

  X.push_back(p0);
  X.push_back(p1);
  X.push_back(p2);
  X.push_back(p3);

  knot_container U;
  
  OpenTissue::spline::compute_chord_length_knot_vector(4, X, U);

  spline_type spline = OpenTissue::spline::make_cubic_interpolation( U, X );

  BOOST_CHECK( spline.get_order() == 4 );

  BOOST_CHECK( spline.get_knot_container().size() == 10 );
  BOOST_CHECK( spline.get_knot_container().size() == (spline.get_control_container().size()+4) );
}

BOOST_AUTO_TEST_SUITE_END();
