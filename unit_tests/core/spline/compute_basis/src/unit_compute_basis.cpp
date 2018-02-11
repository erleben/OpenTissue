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

typedef std::vector<double>                                                           knot_container;

BOOST_AUTO_TEST_SUITE(opentissue_spline_compute_basis);  

BOOST_AUTO_TEST_CASE(test_compute_basis)
{
  knot_container U;

  U.push_back(0.0);
  U.push_back(0.0);
  U.push_back(0.0);  //k = 3
  U.push_back(0.2);
  U.push_back(0.4);
  U.push_back(0.6);
  U.push_back(0.8);  // n = 6  => |P| = 7
  U.push_back(1.0);
  U.push_back(1.0);
  U.push_back(1.0);  // m = 9  => |U| = 10

  // Indices of basis functions belongs to the interval [0..n]

  double const tolerance = 0.00001;

  // Verify that zero other basis functions result in an error
  {
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_basis(2, 0, 0.5, U), std::invalid_argument );
  }

  // Impossible basis function index
  {
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_basis(7, 3, 0.5, U), std::invalid_argument );
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_basis(-1, 3, 0.5, U), std::invalid_argument );
  }


  // Verify that first order basis functions work as intended
  {
    double k1_1 = OpenTissue::spline::detail::compute_basis(0, 1,-0.5, U);
    double k1_2 = OpenTissue::spline::detail::compute_basis(0, 1, 0.0, U);
    double k1_3 = OpenTissue::spline::detail::compute_basis(2, 1,-0.5, U);
    double k1_4 = OpenTissue::spline::detail::compute_basis(2, 1, 0.0, U);
    double k1_5 = OpenTissue::spline::detail::compute_basis(2, 1, 0.1, U);
    double k1_6 = OpenTissue::spline::detail::compute_basis(2, 1, 0.2, U);
    double k1_7 = OpenTissue::spline::detail::compute_basis(2, 1, 0.3, U);
    double k1_8 = OpenTissue::spline::detail::compute_basis(6, 1, 1.0, U); // special case for when hitting maximum U value in last basis interval
    double k1_9 = OpenTissue::spline::detail::compute_basis(6, 1, 1.1, U); 

    BOOST_CHECK_CLOSE(k1_1, 0.0, tolerance);
    BOOST_CHECK_CLOSE(k1_2, 0.0, tolerance);
    BOOST_CHECK_CLOSE(k1_3, 0.0, tolerance);
    BOOST_CHECK_CLOSE(k1_4, 1.0, tolerance);
    BOOST_CHECK_CLOSE(k1_5, 1.0, tolerance);
    BOOST_CHECK_CLOSE(k1_6, 0.0, tolerance);
    BOOST_CHECK_CLOSE(k1_7, 0.0, tolerance);
    BOOST_CHECK_CLOSE(k1_8, 1.0, tolerance);
    BOOST_CHECK_CLOSE(k1_9, 0.0, tolerance);
  }

  // Verify higher order basis functions
  {
    double k3_1 = OpenTissue::spline::detail::compute_basis(2, 3, 0.5, U);
    double k3_2 = OpenTissue::spline::detail::compute_basis(3, 3, 0.5, U);
    double k3_3 = OpenTissue::spline::detail::compute_basis(4, 3, 0.5, U);

    BOOST_CHECK_CLOSE(k3_1, 1.0/8.0, tolerance);
    BOOST_CHECK_CLOSE(k3_2, 6.0/8.0, tolerance);
    BOOST_CHECK_CLOSE(k3_3, 1.0/8.0, tolerance);
  }
}

BOOST_AUTO_TEST_SUITE_END();
