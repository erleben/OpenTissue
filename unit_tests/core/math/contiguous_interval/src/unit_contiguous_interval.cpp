//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_compute_contiguous_angle_interval.h>
#include <vector>


#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_contiguous_interval);

BOOST_AUTO_TEST_CASE(zero_to_two_pi_intervals)
{
  // precomputed interval end-points 

  double const two_pi = OpenTissue::math::detail::pi<double>()*2.0;
  double const pi_sixth = OpenTissue::math::detail::pi<double>()/6.0;
  double const pi_third = OpenTissue::math::detail::pi<double>()/3.0;

  double theta[12];
  theta[0] = 0.0;
  for(size_t i=1;i<12;++i)
    theta[i] = theta[i-1] + pi_sixth;

  // number of samples
  size_t N = 100u; 

  // distance between samples
  double dt = pi_third/(N-1u);

  // container for angle samples
  std::vector<double> angles;
  angles.resize( N );



  // quater sized test is interval  [ (11 pi/6)  .. (pi/6) ]
  {
    double t = theta[11];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
      if(t>two_pi)
        t -= two_pi;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[11],       0.01 );
    BOOST_CHECK_CLOSE(  t_max, two_pi+pi_sixth, 0.01 );
  }
  // test interval is  [ (2 pi/6)  .. (4 pi/6) ]
  {
    double t = theta[2];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[2], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[4], 0.01 );
  }
  // test interval is  [ (5 pi/6)  .. (7 pi/6) ]
  {
    double t = theta[5];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[5], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[7], 0.01 );
  }
  // test interval is  [ (8 pi/6)  .. (10 pi/6) ]
  {
    double t = theta[8];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[8], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[10], 0.01 );
  }

  // Now we want bigger intervals, so distance between samples is changed
  dt = (10.0*pi_sixth)/(N-1u);

  // test interval is  [ (10 pi/6)  .. (8 pi/6) ]
  {
    double t = theta[10];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
      if(t>two_pi)
        t -= two_pi;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[10], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[8]+two_pi, 0.01 );
  }

  // test interval is  [ (4 pi/6)  .. (2 pi/6) ]
  {
    double t = theta[4];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
      if(t>two_pi)
        t -= two_pi;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[4], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[2]+two_pi, 0.01 );
  }
}









BOOST_AUTO_TEST_CASE(minus_pi_to_plus_pi_intervals)
{
  // precomputed interval end-points 

  double const pi = OpenTissue::math::detail::pi<double>();
  double const two_pi = OpenTissue::math::detail::pi<double>()*2.0;
  double const pi_sixth = OpenTissue::math::detail::pi<double>()/6.0;
  double const pi_third = OpenTissue::math::detail::pi<double>()/3.0;

  double theta[12];
  theta[0] = -pi;
  for(size_t i=1;i<12;++i)
    theta[i] = theta[i-1] + pi_sixth;

  // number of samples
  size_t N = 100u; 

  // distance between samples
  double dt = pi_third/(N-1u);

  // container for angle samples
  std::vector<double> angles;
  angles.resize( N );



  // quater sized test is interval  [ (-pi/6)  .. (pi/6) ]
  {
    double t = theta[5];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[5]+two_pi,       0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[7]+two_pi, 0.01 );
  }
  // test interval is  [ (2 pi/6)  .. (4 pi/6) ]
  {
    double t = theta[8];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[8], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[10], 0.01 );
  }
  // test interval is  [ (5 pi/6)  .. (-5 pi/6) ]
  {
    double t = theta[11];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
      if(t>pi)
        t -= two_pi;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[11], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[1]+two_pi, 0.01 );
  }
  // test interval is  [ (-4 pi/6)  .. (-2 pi/6) ]
  {
    double t = theta[2];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[2]+two_pi, 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[4]+two_pi, 0.01 );
  }

  // Now we want bigger intervals, so distance between samples is changed
  dt = (10.0*pi_sixth)/(N-1u);

  // test interval is  [ (-2 pi/6)  .. (-4 pi/6) ]
  {
    double t = theta[4];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
      if(t>pi)
        t -= two_pi;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[4]+two_pi, 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[2]+two_pi+two_pi, 0.01 );
  }

  // test interval is  [ (4 pi/6)  .. (2 pi/6) ]
  {
    double t = theta[10];
    for(size_t i = 0;i < N;++i)
    {
      angles[i] = t;
      t += dt;
      if(t>pi)
        t -= two_pi;
    }
    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, theta[10], 0.01 );
    BOOST_CHECK_CLOSE(  t_max, theta[8]+two_pi, 0.01 );
  }
}




BOOST_AUTO_TEST_CASE(real_data_test)
{
  // precomputed interval end-points 
  double const two_pi = OpenTissue::math::detail::pi<double>()*2.0;
  double const four_pi = OpenTissue::math::detail::pi<double>()*4.0;

  // container for angle samples
  std::vector<double> angles;
  angles.resize( 84 );

  size_t i = 0u;

  angles[i++] = 0.3;
  angles[i++] = 0.4;
  angles[i++] = 0.5;
  angles[i++] = 0.6;
  angles[i++] = 0.7;
  angles[i++] = 0.8;
  angles[i++] = 0.9;
  angles[i++] = 1.0;
  angles[i++] = 1.1;
  angles[i++] = 1.2;
  angles[i++] = 1.3;
  angles[i++] = 1.4;
  angles[i++] = 1.5;
  angles[i++] = 1.6;
  angles[i++] = 1.7;
  angles[i++] = 1.8;
  angles[i++] = 1.9;
  angles[i++] = 2.0;
  angles[i++] = 2.1;
  angles[i++] = 2.2;
  angles[i++] = 2.3;
  angles[i++] = 2.4;
  angles[i++] = 2.5;
  angles[i++] = 2.6;
  angles[i++] = 2.7;
  angles[i++] = 2.8;
  angles[i++] = 2.9;
  angles[i++] = 3.0;
  angles[i++] = 3.1;
  angles[i++] = -3.1;
  angles[i++] = -3.0;
  angles[i++] = -2.9;
  angles[i++] = -2.8;
  angles[i++] = -2.7;
  angles[i++] = -2.6;
  angles[i++] = -2.5;
  angles[i++] = -2.6;
  angles[i++] = -2.7;
  angles[i++] = -2.8;
  angles[i++] = -2.9;
  angles[i++] = -3.0;
  angles[i++] = -3.1;
  angles[i++] = 3.1;
  angles[i++] = 3.0;
  angles[i++] = 2.9;
  angles[i++] = 2.8;
  angles[i++] = 2.7;
  angles[i++] = 2.6;
  angles[i++] = 2.5;
  angles[i++] = 2.4;
  angles[i++] = 2.3;
  angles[i++] = 2.2;
  angles[i++] = 2.1;
  angles[i++] = 2.0;
  angles[i++] = 1.9;
  angles[i++] = 1.8;
  angles[i++] = 1.7;
  angles[i++] = 1.6;
  angles[i++] = 1.5;
  angles[i++] = 1.4;
  angles[i++] = 1.3;
  angles[i++] = 1.2;
  angles[i++] = 1.1;
  angles[i++] = 1.0;
  angles[i++] = 0.9;
  angles[i++] = 0.8;
  angles[i++] = 0.7;
  angles[i++] = 0.6;
  angles[i++] = 0.5;
  angles[i++] = 0.4;
  angles[i++] = 0.3;
  angles[i++] = 0.2;
  angles[i++] = 0.1;
  angles[i++] = 0.0;
  angles[i++] = -0.1;
  angles[i++] = -0.2;
  angles[i++] = -0.3;
  angles[i++] = -0.4;
  angles[i++] = -0.5;
  angles[i++] = -0.6;
  angles[i++] = -0.7;
  angles[i++] = -0.8;
  angles[i++] = -0.9;
  angles[i++] = -1.0;

  BOOST_CHECK( i == angles.size() );

    double t_min, t_max;
    OpenTissue::math::compute_contiguous_angle_interval( angles.begin(), angles.end(), t_min, t_max );
    BOOST_CHECK_CLOSE(  t_min, -1.0+two_pi,       0.01 );
    BOOST_CHECK_CLOSE(  t_max, -2.5+four_pi, 0.01 );
}




BOOST_AUTO_TEST_SUITE_END();
