//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_euler_angles.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

using namespace OpenTissue;


void do_test( double const & phi_in, double const & psi_in, double const & theta_in)
{
  OpenTissue::math::Quaternion<double> Q_in;
  OpenTissue::math::Quaternion<double> Q_out;
  OpenTissue::math::Quaternion<double> identity;
  OpenTissue::math::Quaternion<double> Qz1;
  OpenTissue::math::Quaternion<double> Qy;
  OpenTissue::math::Quaternion<double> Qz2;

  double const too_small = 10e-7;

  double phi_out   = 0.0;
  double psi_out   = 0.0;
  double theta_out = 0.0;

  Qz1.Rz(theta_in);
  Qy.Ry(psi_in);
  Qz2.Rz(phi_in);

  Q_in = OpenTissue::math::prod( Qz2 , OpenTissue::math::prod( Qy , Qz1) );

  OpenTissue::math::ZYZ_euler_angles(Q_in,phi_out,psi_out,theta_out);

  if(psi_in > 0.0)
  {
    // we only want to do this if we are not in a gimbal lock
    Qz1.Rz(theta_out);
    Qy.Ry(psi_out);
    Qz2.Rz(phi_out);
    Q_out = OpenTissue::math::prod( Qz2 , OpenTissue::math::prod( Qy , Qz1) );
    identity = OpenTissue::math::prod( OpenTissue::math::conj(Q_out), Q_in );

    double const s = fabs( fabs(identity.s())  - 1.0 );
    double const v0 = fabs(identity.v()(0));
    double const v1 = fabs(identity.v()(1));
    double const v2 = fabs(identity.v()(2));

    BOOST_CHECK( s < too_small);
    BOOST_CHECK( v0 < too_small);
    BOOST_CHECK( v1 < too_small);
    BOOST_CHECK( v2 < too_small);

    double const dphi = fabs(phi_in - phi_out);
    double const dpsi = fabs(psi_in - psi_out);
    double const dtheta = fabs(theta_in - theta_out);
    BOOST_CHECK( dphi < too_small);
    BOOST_CHECK( dpsi < too_small);
    BOOST_CHECK( dtheta < too_small);
  }
  else
  {
    // In gimbal lock phi and theta behaves strangely
    BOOST_CHECK_CLOSE( 0.0, theta_out, 0.01);
    double const dpsi = fabs(psi_out);
    BOOST_CHECK( dpsi < too_small);

    double new_phi = phi_in + theta_in;
    double const pi     = 3.1415926535897932384626433832795;
    double const two_pi = 2.0*pi;
    while(new_phi>pi) new_phi -= two_pi;
    while(new_phi<-pi) new_phi += two_pi;

    double const dphi = fabs(new_phi - phi_out);
    BOOST_CHECK( dphi < too_small);
  }
}

BOOST_AUTO_TEST_SUITE(opentissue_math_euler_angles);

BOOST_AUTO_TEST_CASE(ZYZ)
{
  size_t N = 15;

  double const pi     = 3.1415926535897932384626433832795;
  double const two_pi = 2.0*pi;
  double const delta = (two_pi)/(N-1);

  double phi = -pi+delta;
  for(;phi<pi;)
  {
    double psi = 0.0;
    for(;psi<pi;)
    {
      double theta = -pi+delta;
      for(;theta<pi;)
      {
        do_test( phi, psi, theta );
        theta += delta;
      }
      psi += delta;
    }
    phi += delta;
  }
}


BOOST_AUTO_TEST_SUITE_END();
