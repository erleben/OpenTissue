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

BOOST_AUTO_TEST_SUITE(opentissue_spline_insert_knot);

BOOST_AUTO_TEST_CASE(test_insert_knot)
{
  knot_container U;

  U.push_back(0.0);
  U.push_back(0.0);
  U.push_back(0.0);
  U.push_back(0.0);  //k = 4
  U.push_back(1.0);
  U.push_back(2.0);
  U.push_back(3.0);  // n = 6  => |P| = 7
  U.push_back(4.0);
  U.push_back(4.0);
  U.push_back(4.0);  
  U.push_back(4.0);  // m = 10  => |U| = 11

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

  spline_type spline(4,U,P);

  double const tolerance = 0.00001;
  double const u = 2.5;

  knot_container const & UU = spline.get_knot_container();
  point_container const & PP = spline.get_control_container();

  //--- Before insertion
  BOOST_CHECK_CLOSE(UU[0], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[1], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[2], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[3], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[4], 1.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[5], 2.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[6], 3.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[7], 4.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[8], 4.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[9], 4.0,  tolerance);
  BOOST_CHECK_CLOSE(UU[10], 4.0,  tolerance);
  BOOST_CHECK( UU.size() == 11 );
  BOOST_CHECK( PP.size() == 7 );

  //--- After insertion
  OpenTissue::spline::insert_knot(u,spline);

  knot_container const & UUU = spline.get_knot_container();
  point_container const & PPP = spline.get_control_container();

  BOOST_CHECK_CLOSE(UUU[0], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[1], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[2], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[3], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[4], 1.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[5], 2.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[6], 2.5,  tolerance);
  BOOST_CHECK_CLOSE(UUU[7], 3.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[8], 4.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[9], 4.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[10], 4.0,  tolerance);
  BOOST_CHECK_CLOSE(UUU[11], 4.0,  tolerance);

  for(int i=0;i<8;++i)
    BOOST_CHECK_CLOSE(PPP[i](1), 0.0,  tolerance);

  BOOST_CHECK_CLOSE(PPP[0](0), 0.0,  tolerance);
  BOOST_CHECK_CLOSE(PPP[1](0), 1.0,  tolerance);
  BOOST_CHECK_CLOSE(PPP[2](0), 2.0,  tolerance);
  BOOST_CHECK_CLOSE(PPP[3](0), 2.833333333333333,  tolerance);
  BOOST_CHECK_CLOSE(PPP[4](0), 3.5,  tolerance);
  BOOST_CHECK_CLOSE(PPP[5](0), 4.25,  tolerance);
  BOOST_CHECK_CLOSE(PPP[6](0), 5.0,  tolerance);
  BOOST_CHECK_CLOSE(PPP[7](0), 6.0,  tolerance);

  BOOST_CHECK( UUU.size() == 12 );
  BOOST_CHECK( PPP.size() == 8 );
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

  double u = 0.5;

  OpenTissue::spline::insert_knot(u,bezier);
  OpenTissue::spline::insert_knot(u,bezier);
  OpenTissue::spline::insert_knot(u,bezier);
  OpenTissue::spline::insert_knot(u,bezier);

  vector_type m0 = 0.5*(p1+p0);
  vector_type m1 = 0.5*(p2+p1);
  vector_type m2 = 0.5*(p3+p2);
  vector_type n0 = 0.5*(m1+m0);
  vector_type n1 = 0.5*(m2+m1);
  vector_type q0 = 0.5*(n0+n1);

  knot_container  const & newU = bezier.get_knot_container();
  point_container const & newP = bezier.get_control_container();

  BOOST_CHECK_CLOSE(newU[0], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(newU[1], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(newU[2], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(newU[3], 0.0,  tolerance);
  BOOST_CHECK_CLOSE(newU[4], 0.5,  tolerance);
  BOOST_CHECK_CLOSE(newU[5], 0.5,  tolerance);
  BOOST_CHECK_CLOSE(newU[6], 0.5,  tolerance);
  BOOST_CHECK_CLOSE(newU[7], 0.5,  tolerance);
  BOOST_CHECK_CLOSE(newU[8], 1.0,  tolerance);
  BOOST_CHECK_CLOSE(newU[9], 1.0,  tolerance);
  BOOST_CHECK_CLOSE(newU[10], 1.0,  tolerance);
  BOOST_CHECK_CLOSE(newU[11], 1.0,  tolerance);
  BOOST_CHECK_CLOSE(newP[0](0), p0(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[0](1), p0(1),  tolerance);
  BOOST_CHECK_CLOSE(newP[1](0), m0(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[1](1), m0(1),  tolerance);
  BOOST_CHECK_CLOSE(newP[2](0), n0(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[2](1), n0(1),  tolerance);
  BOOST_CHECK_CLOSE(newP[3](0), q0(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[3](1), q0(1),  tolerance);
  BOOST_CHECK_CLOSE(newP[4](0), q0(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[4](1), q0(1),  tolerance);
  BOOST_CHECK_CLOSE(newP[5](0), n1(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[5](1), n1(1),  tolerance);
  BOOST_CHECK_CLOSE(newP[6](0), m2(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[6](1), m2(1),  tolerance);
  BOOST_CHECK_CLOSE(newP[7](0), p3(0),  tolerance);
  BOOST_CHECK_CLOSE(newP[7](1), p3(1),  tolerance);
}

BOOST_AUTO_TEST_SUITE_END();
