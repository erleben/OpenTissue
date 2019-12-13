
//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interval/interval_boost_interval_type_traits.h>
#include <OpenTissue/core/math/interval/interval.h>
#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/utility/utility_timer.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

#include <iostream>

using namespace boost::numeric;
using namespace OpenTissue;


// Performance Timings: Centrino Duo, 2GB RAM, .NET 2005
//
//    interval demo started
//    OT type
//        test took 1.74157 seconds
//    boost default type
//        test took 19.701 seconds
//    boost default type with unprotect
//        test took 0.940387 seconds
//    boost smart type
//        test took 0.794767 seconds
//    boost smart type with unprotect
//        test took 0.796035 seconds
//
//
// Linux performance numbers:
//
//  henrik@blackmonster ~/Work/OpenTissue/optimized_test/demos/opengl/interval $ g++ -DHAVE_CONFIG_H -I. -I../../../../demos/opengl/interval -I../../..  -I../../../.. -I../../../../externals/include -O3 -march=pentium4 -fomit-frame-pointer -fwhole-program -pipe -DNDEBUG -o interval ../../../../demos/opengl/interval/src/main.cpp
//  henrik@blackmonster ~/Work/OpenTissue/optimized_test/demos/opengl/interval $ ./interval
//  interval demo started
//  OT type
//        test took 2.68147 seconds
//  boost default type
//        test took 6.14975 seconds
//  boost default type with unprotect
//        test took 4.89632 seconds
//  boost smart type
//        test took 4.12788 seconds
//  boost smart type with unprotect
//        test took 4.12735 seconds
//
//  henrik@blackmonster ~/Work/OpenTissue/optimized_test/demos/opengl/interval $ uname -a
//  Linux blackmonster 2.6.17-gentoo-r8 #3 SMP Mon Nov 6 22:01:53 CET 2006 i686 Intel(R) Pentium(R) 4 CPU 1.60GHz GenuineIntel GNU/Linux
//


template<typename interval_type>
void interval_compile_testing()
{
  typedef typename interval_type::base_type    real_type;
  typedef OpenTissue::math::Vector3<real_type>       vector3_type;
  typedef OpenTissue::math::Vector3<interval_type>   vector3_interval_type;
  typedef OpenTissue::math::Matrix3x3<interval_type> matrix3x3_interval_type;

  //--- implicit type conversions
  interval_type i1;
  interval_type i2(5);
  interval_type i3(5.0);
  interval_type i4(5.0f);
  interval_type i5(5,6);
  interval_type i6(5.0,6.0);
  interval_type i7(5.0f,6.0f);
  interval_type A(-1.0,1.0);
  interval_type B(0.0,1.0);
  interval_type C(-1.0,0.0);
  double s = 1.0;
  //--- arithmetic testing
  A += B;
  A = B + C;
  A -= B;
  A = B - C;
  B = -A;
  A *= B;
  A = B*C;
  A = C*B;
  A /= B;
  A = B/C;
  A = C/B;
  A *= s;
  A = B*s;
  //A = s*B; // do not exist!!!
  //--- only OpenTissue::interval
  //A.clear();
  //A.is_valid();  
  //  A.get_abs_lower();
  //  A.get_abs_upper();
  //  A[-2];
  //  A[0];
  //  A[1];
  //  A[2];
  //  A(-2);
  //  A(0);
  //  A(1);
  //  A(2);
  //--- assignment testing
  A.assign( -1.0,  1.0);
  B.assign( -3.0, -2.0);
  C.assign(  2.0,  4.0);
  //--- logical/comparison testing
  bool cmp1 = (A==B);
  bool cmp2 = (A!=B);
  bool cmp3 = (C<B) ;
  bool cmp4 = (C>B) ;
  bool cmp5 = (C<=B);
  bool cmp6 = (C>=B);

  cmp1 = cmp2 | cmp3 | cmp4 | cmp5 | cmp6; // To get rid of compiler warning: unused variable
  
  //--- intersect testing
  A = intersect(B,C);    
  B = interval_type(-2,-1);
  C = interval_type(1,2);
  A = intersect(B,C);
  A = intersect(C,B);
  B = interval_type(-2,1);
  C = interval_type(-1,2);
  A = intersect(B,C);
  A = intersect(C,B);
  B = interval_type(1,-1);
  C = interval_type(1,1);
  A = intersect(B,C);
  A = intersect(C,B);
}


template<typename interval_type>
void performance_test(interval_type const & i)
{
  using namespace OpenTissue::math::interval;

  OpenTissue::utility::Timer<double> watch;
  watch.start();
  typedef double                               real_type;
  typedef OpenTissue::math::Vector3<real_type>       vector3_type;
  typedef OpenTissue::math::Vector3<interval_type>   vector3_interval_type;
  typedef OpenTissue::math::Matrix3x3<interval_type> matrix3x3_interval_type;
  interval_type A(-1,1);
  matrix3x3_interval_type IM(A,A,A,A,A,A,A,A,A);
  vector3_interval_type IV(A,A,A);
  vector3_type V(1,2,3);
  for(unsigned int i=0u;i<10000000u;++i)
    IV = IM * V + (IV*0.5);
  watch.stop();
  BOOST_TEST_MESSAGE( typeid(interval_type).name() << " took " << watch() << " sec." );
}

BOOST_AUTO_TEST_SUITE(opentissue_math_interval);

  BOOST_AUTO_TEST_CASE(opentissue_testing)
  {
    typedef OpenTissue::math::interval::Interval<double> interval_type;
    performance_test(interval_type());
    void (*ptr) () = 0;
    ptr = &interval_compile_testing< interval_type >;
  }

  BOOST_AUTO_TEST_CASE(boost_default_testing)
  {
    typedef boost::numeric::interval<double >      interval_type;
    performance_test(interval_type());
    void (*ptr) () = 0;
    ptr = &interval_compile_testing< interval_type >;
  }

  BOOST_AUTO_TEST_CASE(boost_default_with_unprotect_testing)
  {
    typedef boost::numeric::interval<double >      interval_type;
    interval_type::traits_type::rounding rnd;
    typedef  boost::numeric::interval_lib::unprotect<interval_type>::type R;
    performance_test(R());
    void (*ptr) () = 0;
    ptr = &interval_compile_testing< interval_type >;
  }

  BOOST_AUTO_TEST_CASE(boost_smart_type_testing)
  {
    typedef OpenTissue::math::interval::BoostIntervalTypeTraits<double>::interval_type interval_type;  
    performance_test(interval_type());
    void (*ptr) () = 0;
    ptr = &interval_compile_testing< interval_type >;
  }

  BOOST_AUTO_TEST_CASE(boost_smart_type_with_unprotect_testing)
  {
    typedef OpenTissue::math::interval::BoostIntervalTypeTraits<double>::interval_type interval_type;
    interval_type::traits_type::rounding rnd;

    rnd;
    
    typedef  boost::numeric::interval_lib::unprotect<interval_type>::type R;
    performance_test(R());
    void (*ptr) () = 0;
    ptr = &interval_compile_testing< interval_type >;
  }

BOOST_AUTO_TEST_SUITE_END();
