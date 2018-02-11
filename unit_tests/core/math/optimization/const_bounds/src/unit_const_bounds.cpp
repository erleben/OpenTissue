//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

#include <OpenTissue/core/math/optimization/optimization_make_constant_bounds.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

template<typename bound_function>
inline void test_vector_bounds( bound_function const & B)
{
  double tol =0.01;

  ublas::vector<double> x;


  BOOST_CHECK_CLOSE( B(x,0) , 1.0 , tol );

  BOOST_CHECK( B.partial_begin(0) == B.partial_end(0) );
  BOOST_CHECK( B.partial_begin(1) == B.partial_end(1) );
  BOOST_CHECK( B.partial_begin(2) == B.partial_end(2) );
  BOOST_CHECK( B.partial_begin(3) == B.partial_end(3) );
  BOOST_CHECK( B.partial_begin(4) == B.partial_end(4) );
  BOOST_CHECK( B.partial_begin(5) == B.partial_end(5) );
  BOOST_CHECK( B.partial_begin(6) == B.partial_end(6) );
  BOOST_CHECK( B.partial_begin(7) == B.partial_end(7) );
  BOOST_CHECK( B.partial_begin(8) == B.partial_end(8) );
  BOOST_CHECK( B.partial_begin(9) == B.partial_end(9) );

  BOOST_CHECK_CLOSE( B(x,1) , 2.0 , tol );
  BOOST_CHECK_CLOSE( B(x,2) , 3.0 , tol );
  BOOST_CHECK_CLOSE( B(x,3) , 4.0 , tol );
  BOOST_CHECK_CLOSE( B(x,4) , 5.0 , tol );
  BOOST_CHECK_CLOSE( B(x,5) , 6.0 , tol );
  BOOST_CHECK_CLOSE( B(x,6) , 7.0 , tol );
  BOOST_CHECK_CLOSE( B(x,7) , 8.0 , tol );
  BOOST_CHECK_CLOSE( B(x,8) , 9.0 , tol );
  BOOST_CHECK_CLOSE( B(x,9) , 10.0 , tol );

}

template<typename bound_function>
inline void test_scalar_bounds( bound_function const & B)
{
  double tol =0.01;
  ublas::vector<double> x;

  BOOST_CHECK( B.partial_begin(0) == B.partial_end(0) );
  BOOST_CHECK( B.partial_begin(1) == B.partial_end(1) );
  BOOST_CHECK( B.partial_begin(2) == B.partial_end(2) );
  BOOST_CHECK( B.partial_begin(3) == B.partial_end(3) );
  BOOST_CHECK( B.partial_begin(4) == B.partial_end(4) );
  BOOST_CHECK( B.partial_begin(5) == B.partial_end(5) );

  BOOST_CHECK_CLOSE( B(x,0) , 2.5 , tol );
  BOOST_CHECK_CLOSE( B(x,1) , 2.5 , tol );
  BOOST_CHECK_CLOSE( B(x,2) , 2.5 , tol );
  BOOST_CHECK_CLOSE( B(x,3) , 2.5 , tol );
  BOOST_CHECK_CLOSE( B(x,4) , 2.5 , tol );
  BOOST_CHECK_CLOSE( B(x,5) , 2.5 , tol );
}



BOOST_AUTO_TEST_SUITE(opentissue_math_big_make_const_bounds);

BOOST_AUTO_TEST_CASE(test_case)
{
  ublas::vector<double> rhs;
  rhs.resize(10,false);
  rhs(0) = 1.0;
  rhs(1) = 2.0;
  rhs(2) = 3.0;
  rhs(3) = 4.0;
  rhs(4) = 5.0;
  rhs(5) = 6.0;
  rhs(6) = 7.0;
  rhs(7) = 8.0;
  rhs(8) = 9.0;
  rhs(9) = 10.0;

  test_vector_bounds( OpenTissue::math::optimization::make_constant_bounds( rhs ));

  double value = 2.5;

  test_scalar_bounds( OpenTissue::math::optimization::make_constant_bounds( value, 10 ) );
}

BOOST_AUTO_TEST_SUITE_END();
