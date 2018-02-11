//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

template<typename math_types>
void compile_test_math_types()
{
  typedef typename math_types::index_type         index_type;
  typedef typename math_types::real_type          real_type;
  typedef typename math_types::vector3_type       vector3_type;
  typedef typename math_types::matrix3x3_type     matrix3x3_type;
  typedef typename math_types::quaternion_type    quaternion_type;
  typedef typename math_types::coordsys_type      coordsys_type;
  typedef typename math_types::index_vector3_type index_vector3_type;
  typedef typename math_types::value_traits       value_traits;

  for(index_type i=0;i<10;++i);  // micky: right :|

  vector3_type     v;
  matrix3x3_type   m;
  quaternion_type  q;
  coordsys_type    c;
  index_vector3_type iv;

  real_type s1 = value_traits::zero();
  real_type s2 = value_traits::one();
  real_type s3 = value_traits::two();
  real_type s4 = value_traits::pi();
  real_type s5 = value_traits::pi_2();
  real_type s6 = value_traits::infinity();
  real_type s7 = value_traits::degree();
  real_type s8 = value_traits::radian();

  BOOST_CHECK( s1 == value_traits::zero() );
  BOOST_CHECK( s2 == value_traits::one() );
  BOOST_CHECK( s3 == value_traits::two() );
  BOOST_CHECK( s4 == value_traits::pi() );
  BOOST_CHECK( s5 == value_traits::pi_2() );
  BOOST_CHECK( s6 == value_traits::infinity() );
  BOOST_CHECK( s7 == value_traits::degree() );
  BOOST_CHECK( s8 == value_traits::radian() );
}


BOOST_AUTO_TEST_SUITE(opentissue_math_basic_math_types);

    BOOST_AUTO_TEST_CASE(type_and_member_compile_test)
    {
      //--- Compile testing that we can instantiate most common types and access members
      typedef OpenTissue::math::BasicMathTypes<float ,      size_t> type1;
      typedef OpenTissue::math::BasicMathTypes<double,      size_t> type2;
      typedef OpenTissue::math::BasicMathTypes<float ,unsigned int> type3;
      typedef OpenTissue::math::BasicMathTypes<double,unsigned int> type4;
      typedef OpenTissue::math::BasicMathTypes<float ,         int> type5;
      typedef OpenTissue::math::BasicMathTypes<double,         int> type6;

      void (*ptr1)() = &(compile_test_math_types<type1>);
      void (*ptr2)() = &(compile_test_math_types<type2>);
      void (*ptr3)() = &(compile_test_math_types<type3>);
      void (*ptr4)() = &(compile_test_math_types<type4>);
      void (*ptr5)() = &(compile_test_math_types<type5>);
      void (*ptr6)() = &(compile_test_math_types<type6>);

      ptr1 = 0;
      ptr2 = 0;
      ptr3 = 0;
      ptr4 = 0;
      ptr5 = 0;
      ptr6 = 0;
    }

BOOST_AUTO_TEST_SUITE_END();
