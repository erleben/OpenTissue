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
typedef math_types::matrix_type                                matrix_type;
typedef std::vector<double>                                    knot_container;

BOOST_AUTO_TEST_SUITE(opentissue_spline_init_m_table);  

BOOST_AUTO_TEST_CASE(test_init_m_table)
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

  // u-parameter at half-way
  {
    double const u = 2.5;

    matrix_type M;
    BOOST_CHECK_NO_THROW( OpenTissue::spline::detail::initialize_m_table(4, u, 4, U, M) );

    // Test content of M.
    BOOST_CHECK_CLOSE(M(0,0), 1.0,     tolerance);
    BOOST_CHECK_CLOSE(M(0,1), 1.0/2.0, tolerance);
    BOOST_CHECK_CLOSE(M(0,2), 1.0/8.0, tolerance);
    BOOST_CHECK_CLOSE(M(1,0), 1.0,     tolerance);
    BOOST_CHECK_CLOSE(M(1,1), 1.0/2.0, tolerance);
    BOOST_CHECK_CLOSE(M(1,2), 6.0/8.0, tolerance);
    BOOST_CHECK_CLOSE(M(2,0), 2.0,     tolerance);
    BOOST_CHECK_CLOSE(M(2,1), 2.0,     tolerance);
    BOOST_CHECK_CLOSE(M(2,2), 1.0/8.0, tolerance);

  }

}

BOOST_AUTO_TEST_SUITE_END();
