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

typedef OpenTissue::spline::MathTypes<double, size_t>                        math_types;
typedef math_types::vector_type                                                    vector_type;
typedef std::vector<double>                                                        knot_container;
typedef std::vector<vector_type>                                                   point_container;
typedef OpenTissue::spline::NUBSpline<knot_container, point_container>             NUBSpline;

knot_container init_knots(int const & M)
{
  knot_container U;
  for(int i=0;i<M;++i)
    U.push_back( i*1.0 );
  return U;
}

point_container init_controls(int const & N)
{
  point_container P;
  for (int i=0;i<N;++i)
  {
    vector_type p(2);
    p(0) = i;
    p(1) = N-i;
    P.push_back(p);
  }
  return P;
}

BOOST_AUTO_TEST_SUITE(opentissue_core_spline_nubs);

BOOST_AUTO_TEST_CASE(test_construction)
{  
  //  No Data Case
  {
    NUBSpline nubspline;

    knot_container & U = nubspline.get_knot_container(); 
    point_container & P = nubspline.get_control_container();
    BOOST_CHECK( U.size() == 0);
    BOOST_CHECK( P.size() == 0);

    knot_container const & UU = nubspline.get_knot_container(); 
    point_container const & PP = nubspline.get_control_container();
    BOOST_CHECK( UU.size() == 0);
    BOOST_CHECK( PP.size() == 0);

    BOOST_CHECK_EQUAL( nubspline.get_order() , 0u);
    BOOST_CHECK_EQUAL( nubspline.get_dimension(), 0u);
  } 
  // The Only Legal Data case
  {
    knot_container U = init_knots(10);
    point_container P = init_controls(7);
    NUBSpline nubspline;
    BOOST_CHECK_NO_THROW( nubspline = NUBSpline(3,U,P) );

    BOOST_CHECK_EQUAL( nubspline.get_dimension(), 2u);
    BOOST_CHECK_EQUAL( nubspline.get_order(), 3u);

    knot_container & UU = nubspline.get_knot_container(); 
    point_container & PP = nubspline.get_control_container();
    BOOST_CHECK( UU.size() == 10);
    BOOST_CHECK( PP.size() == 7);

    knot_container const & UUU = nubspline.get_knot_container(); 
    point_container const & PPP = nubspline.get_control_container();
    BOOST_CHECK( UUU.size() == 10);
    BOOST_CHECK( PPP.size() == 7);
  }
  // Wrong order
  {
    knot_container U = init_knots(10);
    point_container P = init_controls(7);
    NUBSpline nubspline;
    BOOST_CHECK_THROW( nubspline = NUBSpline(2,U,P), std::invalid_argument );
  }
  // Illegal order
  {
    knot_container U = init_knots(10);
    point_container P = init_controls(7);
    NUBSpline nubspline;
    BOOST_CHECK_THROW( nubspline = NUBSpline(0,U,P), std::invalid_argument );
  }
  // Forgot knots
  {
    knot_container U;
    point_container P = init_controls(7);
    NUBSpline nubspline;

    BOOST_CHECK_THROW( nubspline = NUBSpline(3,U,P), std::invalid_argument );
  }
  // Forgot controls
  {
    knot_container U = init_knots(10);
    point_container P;
    NUBSpline nubspline;

    BOOST_CHECK_THROW( nubspline = NUBSpline(3,U,P), std::invalid_argument );
  }
  // Wrong number of controls
  {
    knot_container U = init_knots(10);
    point_container P = init_controls(6);
    NUBSpline nubspline;
    BOOST_CHECK_THROW( nubspline = NUBSpline(3,U,P), std::invalid_argument );
  }
  // Wrong number of knots
  {
    knot_container U = init_knots(9);
    point_container P = init_controls(7);
    NUBSpline nubspline;
    BOOST_CHECK_THROW( nubspline = NUBSpline(3,U,P), std::invalid_argument );
  }
}

BOOST_AUTO_TEST_SUITE_END();
