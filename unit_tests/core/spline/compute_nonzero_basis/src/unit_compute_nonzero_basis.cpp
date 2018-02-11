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

BOOST_AUTO_TEST_SUITE(opentissue_spline_nonzero_compute_basis);  

BOOST_AUTO_TEST_CASE(test_compute_nonzero_basis)
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

  // Indices of basis functions belongs to the interval [0..n]

  double const tolerance = 0.00001;

  // Verify if invalid arguments get caught
  {
    vector_type N;
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_nonzero_basis(-1.0, 3, U, N), std::invalid_argument );
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_nonzero_basis( 6.0, 3, U, N), std::invalid_argument );
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_nonzero_basis( 2.5, 0, U, N), std::invalid_argument );
    BOOST_CHECK_THROW( OpenTissue::spline::detail::compute_nonzero_basis( 1.0, 10, U, N), std::invalid_argument );
  }
  // u-parameter at lower bound
  {
    vector_type N;
    double const u = 0.0;
    BOOST_CHECK_NO_THROW( OpenTissue::spline::detail::compute_nonzero_basis(u, 3, U, N) );
    double const & b1 = N(0);
    double const & b2 = N(1);
    double const & b3 = N(2);
    BOOST_CHECK_CLOSE(b1, 1.0, tolerance);
    BOOST_CHECK_CLOSE(b2, 0.0, tolerance);
    BOOST_CHECK_CLOSE(b3, 0.0, tolerance);
  }
  // u-parameter at half-way
  {
    vector_type N;
    double const u = 2.5;
    BOOST_CHECK_NO_THROW( OpenTissue::spline::detail::compute_nonzero_basis(u, 3, U, N) );
    double const & b1 = N(0);
    double const & b2 = N(1);
    double const & b3 = N(2);
    BOOST_CHECK_CLOSE(b1, 1.0/8.0, tolerance);
    BOOST_CHECK_CLOSE(b2, 6.0/8.0, tolerance);
    BOOST_CHECK_CLOSE(b3, 1.0/8.0, tolerance);
  }
  // u-parameter at upper bound
  {
    vector_type N;
    double const u = 5.0;
    BOOST_CHECK_NO_THROW( OpenTissue::spline::detail::compute_nonzero_basis(u, 3, U, N) );
    double const & b1 = N(0);
    double const & b2 = N(1);
    double const & b3 = N(2);
    BOOST_CHECK_CLOSE(b1, 0.0, tolerance);
    BOOST_CHECK_CLOSE(b2, 0.0, tolerance);
    BOOST_CHECK_CLOSE(b3, 1.0, tolerance);
  }
}

BOOST_AUTO_TEST_SUITE_END();
