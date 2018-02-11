//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/geometry/scan_conversion/scan_conversion_fragment_iterator.h>
#include <cmath> // needed for std::fabs

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


BOOST_AUTO_TEST_SUITE(scan_conversion);

BOOST_AUTO_TEST_CASE(triangle_test_case_1)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>                   math_types;
  typedef math_types::vector3_type                                          vector3_type;
  typedef math_types::real_type                                             real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, -5.000000, 0.000000);
  vector3_type v3(0.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.928477 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.988936 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.997785 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999688 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.898734 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.287949 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.491544 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.863848 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.944211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.310211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.782316 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.620421 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.999688 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_2)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>                   math_types;
  typedef math_types::vector3_type                                          vector3_type;
  typedef math_types::real_type                                             real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-4.000000, -5.000000, 0.000000);
  vector3_type v3(0.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.287348 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.957826 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.033315 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999445 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.864291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.305779 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.399376 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.581911 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.152889 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.798752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.877133 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.084291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.472792 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_3)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-3.000000, -5.000000, 0.000000);
  vector3_type v3(0.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.196116 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.980581 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.049938 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.998752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_4)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-2.000000, -5.000000, 0.000000);
  vector3_type v3(0.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_5)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-1.000000, -5.000000, 0.000000);
  vector3_type v3(0.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_6)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(0.000000, -5.000000, 0.000000);
  vector3_type v3(5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.928477 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.988936 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.997785 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999688 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.898734 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.287949 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.491544 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.863848 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.944211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.310211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.782316 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.620421 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.999688 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_7)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(1.000000, -5.000000, 0.000000);
  vector3_type v3(5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.287348 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.957826 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.033315 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999445 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.864291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.305779 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.399376 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.581911 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.152889 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.798752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.877133 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.084291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.472792 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_8)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(2.000000, -5.000000, 0.000000);
  vector3_type v3(5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.196116 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.980581 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.049938 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.998752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_9)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(3.000000, -5.000000, 0.000000);
  vector3_type v3(5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_10)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(4.000000, -5.000000, 0.000000);
  vector3_type v3(5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -5 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_11)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, -5.000000, 0.000000);
  vector3_type v3(5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.691714 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.207514 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.691714 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.890871 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.089087 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.445435 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.445435 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.089087 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.890871 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.999688 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.948209 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.031607 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.316070 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.706665 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.035333 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.706665 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.316070 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.031607 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.948209 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_12)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, -4.000000, 0.000000);
  vector3_type v3(5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.957826 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.287348 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.995037 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.700140 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.140028 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.700140 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.999445 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.033315 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.893534 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.044677 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.446767 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.446767 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.044677 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.893534 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_13)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, -3.000000, 0.000000);
  vector3_type v3(5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.980581 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.196116 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.998752 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.049938 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.892421 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066932 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.446211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.446211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066932 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.892421 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_14)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, -2.000000, 0.000000);
  vector3_type v3(5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.995037 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.700140 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.140028 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.700140 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_15)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, -1.000000, 0.000000);
  vector3_type v3(5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
}

BOOST_AUTO_TEST_CASE(triangle_test_case_16)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, 0.000000, 0.000000);
  vector3_type v3(5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.999688 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.997785 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.988936 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.928477 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.863848 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.491544 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.619754 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.748075 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.287949 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.898734 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.620421 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.782316 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.310211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.944211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.289930 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.956416 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_17)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, 1.000000, 0.000000);
  vector3_type v3(5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.920358 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.316228 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.230089 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.685994 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.707107 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.171499 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.306786 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.948683 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.076696 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.623419 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.107649 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.774442 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.311710 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.215297 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.925464 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.290111 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.052041 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.955577 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_18)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, 2.000000, 0.000000);
  vector3_type v3(5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.991219 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.050189 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.122333 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.744208 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.447214 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.496139 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.372104 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.894427 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.248069 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.291071 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.102909 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.951151 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_19)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, 3.000000, 0.000000);
  vector3_type v3(5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.951151 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.102909 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.291071 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.392232 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.707107 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.588348 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_20)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, 4.000000, 0.000000);
  vector3_type v3(5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_21)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(5.000000, 5.000000, 0.000000);
  vector3_type v3(0.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.782316 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.620421 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.944211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.310211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.491544 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.863848 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.898734 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.287949 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_22)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(4.000000, 5.000000, 0.000000);
  vector3_type v3(0.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.782316 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.620421 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.944211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.310211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.491544 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.863848 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==3 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.898734 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.287949 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_23)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(3.000000, 5.000000, 0.000000);
  vector3_type v3(0.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.877133 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.084291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.472792 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.581911 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.152889 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.798752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==2 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.864291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.305779 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.399376 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_24)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(2.000000, 5.000000, 0.000000);
  vector3_type v3(0.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.877133 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.084291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.472792 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==1 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_25)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(1.000000, 5.000000, 0.000000);
  vector3_type v3(0.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==0 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_26)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(0.000000, 5.000000, 0.000000);
  vector3_type v3(-5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.782316 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.620421 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.944211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.310211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.491544 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.863848 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.898734 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.287949 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_27)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-1.000000, 5.000000, 0.000000);
  vector3_type v3(-5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.782316 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.620421 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.944211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.310211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.491544 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.863848 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.898734 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.287949 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_28)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-2.000000, 5.000000, 0.000000);
  vector3_type v3(-5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.956416 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.289930 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.877133 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.084291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.472792 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.581911 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.152889 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.798752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.864291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.305779 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.399376 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_29)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-3.000000, 5.000000, 0.000000);
  vector3_type v3(-5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.877133 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.084291 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.472792 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.748075 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.619754 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_30)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-4.000000, 5.000000, 0.000000);
  vector3_type v3(-5.000000, 5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_31)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, 5.000000, 0.000000);
  vector3_type v3(-5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 1.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999688 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.316070 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.031607 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.948209 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.706665 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.035333 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.706665 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.948209 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.031607 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.316070 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.997785 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.445435 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.089087 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.890871 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.890871 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.089087 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.445435 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.988936 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.691714 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.207514 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.691714 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.928477 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_32)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, 4.000000, 0.000000);
  vector3_type v3(-5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 1.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.033315 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999445 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.315947 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.042126 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.947841 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.706322 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.047088 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.706322 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.947841 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.042126 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.315947 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.443242 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.132973 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.886484 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.886484 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.132973 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.443242 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.287348 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.957826 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.650945 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.390567 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.650945 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_33)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, 3.000000, 0.000000);
  vector3_type v3(-5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 1.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.049938 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.998752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.315597 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.063119 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.946792 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.705346 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.070535 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.705346 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.946792 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.063119 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.315597 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.196116 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.980581 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.680414 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.272166 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.680414 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_34)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, 2.000000, 0.000000);
  vector3_type v3(-5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 1.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.443242 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.132973 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.886484 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.886484 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.132973 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.443242 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_35)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, 1.000000, 0.000000);
  vector3_type v3(-5.000000, 0.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 1.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.242536 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.970143 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.554700 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.832050 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.832050 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.554700 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-1 );
  BOOST_CHECK( pixel.y() == 0 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.970143 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.242536 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_36)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, 0.000000, 0.000000);
  vector3_type v3(-5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.024992 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999688 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.066519 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.997785 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.289930 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.034768 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.956416 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.148340 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.988936 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.310211 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110611 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.944211 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.620421 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.055305 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.782316 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.371391 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.928477 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.287949 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.330699 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.898734 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.619754 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.237255 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.748075 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.863848 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.110233 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.491544 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_37)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, -1.000000, 0.000000);
  vector3_type v3(-5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.033315 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.999445 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.290111 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.052041 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.955577 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.287348 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.957826 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.311710 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.215297 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.925464 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.623419 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.107649 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.774442 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.306786 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.948683 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.076696 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.685994 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.707107 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.171499 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.920358 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.316228 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.230089 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_38)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, -2.000000, 0.000000);
  vector3_type v3(-5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.049938 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.998752 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.196116 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.980581 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.291071 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.102909 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.951151 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.372104 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.894427 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.248069 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.744208 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.447214 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.496139 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-2 );
  BOOST_CHECK( pixel.y() == -1 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.995037 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_39)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, -3.000000, 0.000000);
  vector3_type v3(-5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.099504 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.995037 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-4 );
  BOOST_CHECK( pixel.y() == -3 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.392232 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.707107 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.588348 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-3 );
  BOOST_CHECK( pixel.y() == -2 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.980581 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 0.196116 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}

BOOST_AUTO_TEST_CASE(triangle_test_case_40)
{
  using std::fabs;

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef math_types::real_type                            real_type;
  typedef OpenTissue::scan_conversion::FragmentIterator<vector3_type>       fragment_iterator;

  vector3_type v1(0.000000, 0.000000, 0.000000);
  vector3_type v2(-5.000000, -4.000000, 0.000000);
  vector3_type v3(-5.000000, -5.000000, 0.000000);
  vector3_type n1(1.000000, 0.000000, 0.000000);
  vector3_type n2(0.000000, 0.100000, 0.000000);
  vector3_type n3(0.000000, 0.000000, 1.000000);
  real_type epsilon = 10e-7;

  fragment_iterator pixel(v1,n1,v2,n2,v3,n3);
  BOOST_CHECK( pixel() );
  BOOST_CHECK( pixel.x() ==-5 );
  BOOST_CHECK( pixel.y() == -4 );
  BOOST_CHECK( fabs( pixel.normal()(0) - 0.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(1) - 1.000000 ) < epsilon );
  BOOST_CHECK( fabs( pixel.normal()(2) - 0.000000 ) < epsilon );
  BOOST_CHECK_NO_THROW( ++pixel );
}
BOOST_AUTO_TEST_SUITE_END();
