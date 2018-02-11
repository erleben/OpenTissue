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

typedef OpenTissue::spline::MathTypes<double, size_t>                           math_types;
typedef math_types::vector_type                                                       vector_type;
typedef std::vector<double>                                                           knot_container;
typedef std::vector<vector_type>                                                      point_container;
typedef OpenTissue::spline::NUBSpline<knot_container, point_container>                NUBSpline;

knot_container init_knots(int const & k, int const & M)
{
  int m = M - 1;
  int const n = m-k;

  knot_container U;

  double knot_value = 0.0;
  int i = 0;
  for(;i<k;++i)
   U.push_back(knot_value);

  for(;i<=n;++i)
  {
    knot_value += 1.0;
     U.push_back(knot_value);
  }
  knot_value += 1.0;
  for(;i<M;++i)
   U.push_back(knot_value);
  return U;
}

BOOST_AUTO_TEST_SUITE(opentissue_spline_compute_knot_span);

BOOST_AUTO_TEST_CASE(test_compute_knot_span)
{
  // Create knot vector: U = 0 0 0 1 2 3 4 5 5 5
  knot_container U = init_knots(3,10);
  {
    double u = -0.5;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 2 );
  }
  {
    double u = 0.0;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 2 );
  }
  {
    double u = 0.5;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 2 );
  }
  {
    double u = 1.0;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 3 );
  }
  {
    double u = 1.5;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 3 );
  }
  {
    double u = 2.0;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 4 );
  }
  {
    double u = 2.5;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 4 );
  }
  {
    double u = 3.0;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 5 );
  }
  {
    double u = 3.5;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 5 );
  }
  {
    double u = 4.0;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 6 );
  }
  {
    double u = 4.5;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 6 );
  }
  {
    double u = 5.0;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 6 );
  }
  {
    double u = 5.5;
    int idx = OpenTissue::spline::detail::compute_knot_span(u,3,U);
    BOOST_CHECK( idx == 6 );
  }
  // Illegal order
  {
    double u = 2.5;
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_knot_span(u,0,U), std::invalid_argument );
  }
  // Too few element in U
  {
    double u = 2.5;
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_knot_span(u,6,U), std::invalid_argument );
  }
}

BOOST_AUTO_TEST_SUITE_END();
