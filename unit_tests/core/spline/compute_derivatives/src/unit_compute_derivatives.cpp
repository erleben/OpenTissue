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


BOOST_AUTO_TEST_SUITE(opentissue_spline_compute_derivatives);

BOOST_AUTO_TEST_CASE(test_compute_derivatives)
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

  double const tolerance = 0.00001;

  point_container dC0;
  point_container dC1;
  point_container dC2;
  point_container dC3;
  point_container dC4;
  point_container dC5;
  point_container dC6;

  double const du = 5.0/6.0;
  double u = 0.0;
  OpenTissue::spline::compute_spline_derivatives(spline, u, 1, dC0, math_types() );
  u += du;
  OpenTissue::spline::compute_spline_derivatives(spline, u, 1, dC1, math_types());
  u += du;
  OpenTissue::spline::compute_spline_derivatives(spline, u, 1, dC2, math_types());
  u += du;
  OpenTissue::spline::compute_spline_derivatives(spline, u, 1, dC3, math_types());
  u += du;
  OpenTissue::spline::compute_spline_derivatives(spline, u, 1, dC4, math_types());
  u += du;
  OpenTissue::spline::compute_spline_derivatives(spline, u, 1, dC5, math_types());
  u += du;
  OpenTissue::spline::compute_spline_derivatives(spline, u, 1, dC6, math_types());

  BOOST_CHECK_CLOSE( dC0[0](1), 0.0, tolerance );
  BOOST_CHECK_CLOSE( dC1[0](1), 0.0, tolerance );
  BOOST_CHECK_CLOSE( dC2[0](1), 0.0, tolerance );
  BOOST_CHECK_CLOSE( dC3[0](1), 0.0, tolerance );
  BOOST_CHECK_CLOSE( dC4[0](1), 0.0, tolerance );
  BOOST_CHECK_CLOSE( dC5[0](1), 0.0, tolerance );
  BOOST_CHECK_CLOSE( dC6[0](1), 0.0, tolerance );

  //BOOST_CHECK( dC0[0](0) > 0.0 ); // why is this zero?
  BOOST_CHECK( dC1[0](0) > 0.0 );
  BOOST_CHECK( dC2[0](0) > 0.0 );
  BOOST_CHECK( dC3[0](0) > 0.0 );
  BOOST_CHECK( dC4[0](0) > 0.0 );
  BOOST_CHECK( dC5[0](0) > 0.0 );
  BOOST_CHECK( dC6[0](0) > 0.0 );

  //std::cout << dC0[0](0) << " " << dC0[0](1) << std::endl;
  //std::cout << dC1[0](0) << " " << dC1[0](1) << std::endl;
  //std::cout << dC2[0](0) << " " << dC2[0](1) << std::endl;
  //std::cout << dC3[0](0) << " " << dC3[0](1) << std::endl;
  //std::cout << dC4[0](0) << " " << dC4[0](1) << std::endl;
  //std::cout << dC5[0](0) << " " << dC5[0](1) << std::endl;
  //std::cout << dC6[0](0) << " " << dC6[0](1) << std::endl;
}


BOOST_AUTO_TEST_CASE(test_bezier_curve_case)
{
  knot_container U;

  U.push_back(0.0);
  U.push_back(0.0);
  U.push_back(0.0);  
  U.push_back(0.0);   
  U.push_back(1.0);
  U.push_back(1.0);
  U.push_back(1.0);  
  U.push_back(1.0);  

  // k = 4
  // n = 3  => |P| = 4
  // m = 7  => |U| = 8

  point_container P;
  vector_type p0(2);  p0(0) = 0.0; p0(1) = 0.0;
  vector_type p1(2);  p1(0) = 1.0; p1(1) = 1.0;
  vector_type p2(2);  p2(0) = 2.0; p2(1) = 1.0;
  vector_type p3(2);  p3(0) = 3.0; p3(1) = 0.0;

  P.push_back(p0);
  P.push_back(p1);
  P.push_back(p2);
  P.push_back(p3);

  spline_type bezier(4,U,P);

  double const tolerance = 10e-15;

  // Now we have constructed the equivalent of a bezier curve
  //
  //   B(t) =     a t^3 +   b t^2 + c t + d
  //   B'(t) =  3 a t^2 + 2 b t   + c 
  //
  //   B(0) = p0  = d
  //   B(1) = p3  = a + b + c + d
  //   B'(0) = 3*(p1-p0) = c
  //   B'(1) = 3*(p1-p0) = 3 a + 2 b + c
  //
  //  working out the equations one would get
  vector_type a = -1*p0 + 3*p1 - 3*p2 + 1*p3;
  vector_type b =  3*p0 - 6*p1 + 3*p2;
  vector_type c = -3*p0 + 3*p1;
  vector_type d =  1*p0;

  double u = 0.0;
  int const N = 11;
  double const du = 1.0 / (N-1);
  for(int i=0;i<N;++i)
  {
    point_container dC;
    // ask for zero'th, first and second order derivatives
    OpenTissue::spline::compute_spline_derivatives(bezier, u, 2, dC, math_types() );

    vector_type q(2);
    q = (u*u*u)*a + (u*u)*b + u*c + d;

    vector_type dq(2);
    dq = 3*(u*u)*a + 2*(u)*b + c;

    vector_type ddq(2);
    ddq = 6*u*a + 2*b;

    BOOST_CHECK( std::fabs( dC[0](0) - q(0) ) < tolerance );
    BOOST_CHECK( std::fabs( dC[0](1) - q(1) ) < tolerance );

    BOOST_CHECK( std::fabs( dC[1](0) - dq(0) ) < tolerance );
    BOOST_CHECK( std::fabs( dC[1](1) - dq(1) ) < tolerance );

    BOOST_CHECK( std::fabs( dC[2](0) - ddq(0) ) < tolerance );
    BOOST_CHECK( std::fabs( dC[2](1) - ddq(1) ) < tolerance );
    u += du;
  }
}

BOOST_AUTO_TEST_SUITE_END();
