//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_gmres.h>


#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

BOOST_AUTO_TEST_SUITE(opentissue_math_big_gmres);

BOOST_AUTO_TEST_CASE(one_size_problem_test)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;
  {
    matrix_type A;
    A.resize(1,1,false);
    vector_type x,b,y;
    x.resize(1,false);
    b.resize(1,false);
    y.resize(1,false);
    b(0) = 4.0;
    A(0,0) = -1.0;
    y(0) = -4.0;
    unsigned int R = 4;
    double eps = 1.000000e-002;
    unsigned int M = 4;
    x.clear();
    OpenTissue::math::big::gmres(A,x,b,M,R,eps);

    double tol = 0.01;
    BOOST_CHECK_CLOSE( real_type( x(0) ), real_type( y(0) ), tol );
  }
  {
    matrix_type A;
    A.resize(1,1,false);
    vector_type x,b,y;
    x.resize(1,false);
    b.resize(1,false);
    y.resize(1,false);
    b(0) = 4.0;
    A(0,0) = 1.0;
    y(0) = 4.0;
    unsigned int R = 4;
    double eps = 1.000000e-002;
    unsigned int M = 4;
    x.clear();
    OpenTissue::math::big::gmres(A,x,b,M,R,eps);
    double tol = 0.01;
    BOOST_CHECK_CLOSE( real_type( x(0) ), real_type( y(0) ), tol );
  }
  {
    matrix_type A;
    A.resize(1,1,false);
    vector_type x,b,y;
    x.resize(1,false);
    b.resize(1,false);
    y.resize(1,false);
    b(0) = -4.0;
    A(0,0) = 1.0;
    y(0) = -4.0;
    unsigned int R = 4;
    double eps = 1.000000e-002;
    unsigned int M = 4;
    x.clear();
    OpenTissue::math::big::gmres(A,x,b,M,R,eps);
    double tol = 0.01;
    BOOST_CHECK_CLOSE( real_type( x(0) ), real_type( y(0) ), tol );
  }
  {
    matrix_type A;
    A.resize(1,1,false);
    vector_type x,b,y;
    x.resize(1,false);
    b.resize(1,false);
    y.resize(1,false);
    b(0) = -4.0;
    A(0,0) = -1.0;
    y(0) = 4.0;
    unsigned int R = 4;
    double eps = 1.000000e-002;
    unsigned int M = 4;
    x.clear();
    OpenTissue::math::big::gmres(A,x,b,M,R,eps);
    double tol = 0.01;
    BOOST_CHECK_CLOSE( real_type( x(0) ), real_type( y(0) ), tol );
  }
}



BOOST_AUTO_TEST_CASE(logic_and_valid_arguments_testing)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;

  // not done yet
}

BOOST_AUTO_TEST_CASE(singular_test)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;  b(1) = 3.000000e+000;  b(2) = 2.000000e+000;  b(3) = 1.000000e+000;
    A(0,0) = 1.000000e+000;  A(0,1) = 2.000000e+000;  A(0,2) = 3.000000e+000;  A(0,3) = 4.000000e+000;
    A(1,0) = 0.000000e+000;  A(1,1) = 1.000000e+000;  A(1,2) = 2.000000e+000;  A(1,3) = 3.000000e+000;
    A(2,0) = 0.000000e+000;  A(2,1) = 0.000000e+000;  A(2,2) = 1.000000e+000;  A(2,3) = 2.000000e+000;
    A(3,0) = 1.000000e+000;  A(3,1) = 2.000000e+000;  A(3,2) = 3.000000e+000;  A(3,3) = 4.000000e+000;
    y(0) = -1.500000e+000;  y(1) = -2.875000e-001;  y(2) = 5.750000e-001;  y(3) = 7.125000e-001;
    unsigned int R = 4;
    double eps = 1.000000e-002;
    unsigned int M = 4;
    BOOST_CHECK_THROW( OpenTissue::math::big::gmres(A,x,b,M,R,eps)        , std::logic_error );  
  }
}

BOOST_AUTO_TEST_CASE(test_case_1)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;
  real_type tol = 5.0;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = -1.450000e+000;	  y(1) = -5.250000e-001;	  y(2) = 4.000000e-001;	  y(3) = 1.325000e+000;	

    size_t R = 0;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = 5.328244e-002;	  y(1) = 4.408904e-002;	  y(2) = 3.489564e-002;	  y(3) = 2.570223e-002;	

    size_t R = 1;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 4 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = 7.142857e-002;	  y(1) = 5.357143e-002;	  y(2) = 3.571429e-002;	  y(3) = 1.785714e-002;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = -1.450000e+000;	  y(1) = -5.250000e-001;	  y(2) = 4.000000e-001;	  y(3) = 1.325000e+000;	

    size_t R = 2;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = 7.142857e-002;	  y(1) = 5.357143e-002;	  y(2) = 3.571429e-002;	  y(3) = 1.785714e-002;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = -1.450000e+000;	  y(1) = -5.250000e-001;	  y(2) = 4.000000e-001;	  y(3) = 1.325000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = 5.360509e-002;	  y(1) = 4.426747e-002;	  y(2) = 3.492986e-002;	  y(3) = 2.559225e-002;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = -1.450000e+000;	  y(1) = -5.250000e-001;	  y(2) = 4.000000e-001;	  y(3) = 1.325000e+000;	

    size_t R = 3;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = -1.450000e+000;	  y(1) = -5.250000e-001;	  y(2) = 4.000000e-001;	  y(3) = 1.325000e+000;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 5.000000e+000;	  A(1,1) = 6.000000e+000;	  A(1,2) = 7.000000e+000;	  A(1,3) = 8.000000e+000;	
    A(2,0) = 9.000000e+000;	  A(2,1) = 1.000000e+001;	  A(2,2) = 1.100000e+001;	  A(2,3) = 1.200000e+001;	
    A(3,0) = 1.300000e+001;	  A(3,1) = 1.400000e+001;	  A(3,2) = 1.500000e+001;	  A(3,3) = 1.600000e+001;	

    y(0) = -1.450000e+000;	  y(1) = -5.250000e-001;	  y(2) = 4.000000e-001;	  y(3) = 1.325000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
}

BOOST_AUTO_TEST_CASE(test_case_2)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;

  real_type tol = 5.0;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = -3.939116e-001;	  y(1) = -5.705575e-001;	  y(2) = 1.178548e+000;	  y(3) = 9.621481e-001;	

    size_t R = 0;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = 1.037950e-001;	  y(1) = -3.667695e-001;	  y(2) = 6.276055e-001;	  y(3) = 6.268646e-001;	

    size_t R = 1;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 4 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = 2.298458e-001;	  y(1) = 1.870824e-001;	  y(2) = 2.920816e-001;	  y(3) = 2.612753e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = 5.397502e-002;	  y(1) = -5.317433e-001;	  y(2) = 7.343428e-001;	  y(3) = 7.529076e-001;	

    size_t R = 2;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = 2.298458e-001;	  y(1) = 1.870824e-001;	  y(2) = 2.920816e-001;	  y(3) = 2.612753e-001;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = -3.939116e-001;	  y(1) = -5.705575e-001;	  y(2) = 1.178548e+000;	  y(3) = 9.621481e-001;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = 1.073667e-001;	  y(1) = -3.730789e-001;	  y(2) = 6.305885e-001;	  y(3) = 6.285444e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = -3.916396e-001;	  y(1) = -5.704548e-001;	  y(2) = 1.175461e+000;	  y(3) = 9.618193e-001;	

    size_t R = 3;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 3 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = -3.939116e-001;	  y(1) = -5.705575e-001;	  y(2) = 1.178548e+000;	  y(3) = 9.621481e-001;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 5.465712e-001;	  b(1) = 4.448802e-001;	  b(2) = 6.945672e-001;	  b(3) = 6.213101e-001;	
    A(0,0) = 4.965524e-001;	  A(0,1) = 8.179743e-001;	  A(0,2) = 3.411936e-001;	  A(0,3) = 8.384960e-001;	
    A(1,0) = 8.997692e-001;	  A(1,1) = 6.602276e-001;	  A(1,2) = 5.340790e-001;	  A(1,3) = 5.680725e-001;	
    A(2,0) = 8.216292e-001;	  A(2,1) = 3.419706e-001;	  A(2,2) = 7.271132e-001;	  A(2,3) = 3.704136e-001;	
    A(3,0) = 6.449104e-001;	  A(3,1) = 2.897259e-001;	  A(3,2) = 3.092902e-001;	  A(3,3) = 7.027399e-001;	

    y(0) = -3.939116e-001;	  y(1) = -5.705575e-001;	  y(2) = 1.178548e+000;	  y(3) = 9.621481e-001;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
}

BOOST_AUTO_TEST_CASE(test_case_3)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;

  real_type tol = 5.0;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.176039e-001;	  y(1) = 1.023936e+000;	  y(2) = 5.194265e-001;	  y(3) = 2.866916e-001;	

    size_t R = 0;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.173759e-001;	  y(1) = 1.022958e+000;	  y(2) = 5.179188e-001;	  y(3) = 2.867674e-001;	

    size_t R = 1;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 4 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 1.854053e-001;	  y(1) = 1.050268e+000;	  y(2) = 2.909857e-001;	  y(3) = 2.704917e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.330048e-001;	  y(1) = 1.020493e+000;	  y(2) = 5.050720e-001;	  y(3) = 2.993282e-001;	

    size_t R = 2;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 1.854053e-001;	  y(1) = 1.050268e+000;	  y(2) = 2.909857e-001;	  y(3) = 2.704917e-001;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.176039e-001;	  y(1) = 1.023936e+000;	  y(2) = 5.194265e-001;	  y(3) = 2.866916e-001;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.175929e-001;	  y(1) = 1.024079e+000;	  y(2) = 5.188623e-001;	  y(3) = 2.866875e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.176531e-001;	  y(1) = 1.023925e+000;	  y(2) = 5.193806e-001;	  y(3) = 2.867320e-001;	

    size_t R = 3;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 2 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.176039e-001;	  y(1) = 1.023936e+000;	  y(2) = 5.194265e-001;	  y(3) = 2.866916e-001;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 1.729561e-001;	  b(1) = 9.797469e-001;	  b(2) = 2.714473e-001;	  b(3) = 2.523293e-001;	
    A(0,0) = 7.948211e-001;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 9.568434e-001;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 5.225903e-001;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 8.801422e-001;	

    y(0) = 2.176039e-001;	  y(1) = 1.023936e+000;	  y(2) = 5.194265e-001;	  y(3) = 2.866916e-001;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
}

BOOST_AUTO_TEST_CASE(test_case_4)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;

  real_type tol = 5.0;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 0;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 1;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 2;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 3;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 0.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 4.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
}

BOOST_AUTO_TEST_CASE(test_case_5)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;

  real_type tol = 5.0;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -2.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 0;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 6.760069e-001;	  y(1) = 2.529694e+000;	  y(2) = 1.686462e+000;	  y(3) = 8.432312e-001;	

    size_t R = 1;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 4 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 1.894737e+000;	  y(1) = 1.421053e+000;	  y(2) = 9.473684e-001;	  y(3) = 4.736842e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -2.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 2;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 1.894737e+000;	  y(1) = 1.421053e+000;	  y(2) = 9.473684e-001;	  y(3) = 4.736842e-001;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -2.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -2.388981e+000;	  y(1) = 3.360331e+000;	  y(2) = 2.240221e+000;	  y(3) = 1.120110e+000;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -2.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 3;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -2.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 0.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 0.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -2.000000e+000;	  y(1) = 3.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    for(size_type i = 0; i < x.size();++i)
      BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
}

BOOST_AUTO_TEST_CASE(test_case_6)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 6.661338e-016;	  y(1) = -1.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 0;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 3 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 1.108327e+000;	  y(1) = 9.376982e-001;	  y(2) = 6.828899e-001;	  y(3) = 3.414450e-001;	

    size_t R = 1;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 4 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 1.161290e+000;	  y(1) = 8.709677e-001;	  y(2) = 5.806452e-001;	  y(3) = 2.903226e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -1.515275e+000;	  y(1) = 6.456212e-001;	  y(2) = 1.380855e+000;	  y(3) = 6.904277e-001;	

    size_t R = 2;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 1.161290e+000;	  y(1) = 8.709677e-001;	  y(2) = 5.806452e-001;	  y(3) = 2.903226e-001;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 6.661338e-016;	  y(1) = -1.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 3 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 1.107136e+000;	  y(1) = 9.385017e-001;	  y(2) = 6.844093e-001;	  y(3) = 3.422047e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 6.661338e-016;	  y(1) = -1.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 3;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 3 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 6.661338e-016;	  y(1) = -1.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 3 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 0.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 0.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 0.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 6.661338e-016;	  y(1) = -1.000000e+000;	  y(2) = 2.000000e+000;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 3 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
}

BOOST_AUTO_TEST_CASE(test_case_7)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef matrix_type::value_type          real_type;
  typedef matrix_type::size_type           size_type;
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 0.000000e+000;	  y(1) = 1.942890e-016;	  y(2) = -2.775558e-016;	  y(3) = 1.000000e+000;	

    size_t R = 0;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 8.954127e-001;	  y(1) = 7.167146e-001;	  y(2) = 5.020614e-001;	  y(3) = 2.601779e-001;	

    size_t R = 1;
    double eps = 0.000000e+000;
    size_t M = 0;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 4 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 9.206963e-001;	  y(1) = 6.905222e-001;	  y(2) = 4.603482e-001;	  y(3) = 2.301741e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -8.834727e-001;	  y(1) = 2.853176e-001;	  y(2) = 6.957702e-001;	  y(3) = 5.374695e-001;	

    size_t R = 2;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 2 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 9.206963e-001;	  y(1) = 6.905222e-001;	  y(2) = 4.603482e-001;	  y(3) = 2.301741e-001;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 0.000000e+000;	  y(1) = 1.942890e-016;	  y(2) = -2.775558e-016;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 1;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 8.952309e-001;	  y(1) = 7.168181e-001;	  y(2) = 5.022643e-001;	  y(3) = 2.603315e-001;	

    size_t R = 1;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 1 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = -9.969868e-004;	  y(1) = 1.445278e-003;	  y(2) = -1.063212e-003;	  y(3) = 1.000311e+000;	

    size_t R = 3;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 3 );
    BOOST_CHECK(  used_outer_iterations == 5 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 1 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 0.000000e+000;	  y(1) = 1.942890e-016;	  y(2) = -2.775558e-016;	  y(3) = 1.000000e+000;	

    size_t R = 4;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
  {
    matrix_type A;
    A.resize(4,4,false);
    vector_type x,b,y;
    x.resize(4,false);
    b.resize(4,false);
    y.resize(4,false);
    b(0) = 4.000000e+000;	  b(1) = 3.000000e+000;	  b(2) = 2.000000e+000;	  b(3) = 1.000000e+000;	
    A(0,0) = 1.000000e+000;	  A(0,1) = 2.000000e+000;	  A(0,2) = 3.000000e+000;	  A(0,3) = 4.000000e+000;	
    A(1,0) = 0.000000e+000;	  A(1,1) = 1.000000e+000;	  A(1,2) = 2.000000e+000;	  A(1,3) = 3.000000e+000;	
    A(2,0) = 0.000000e+000;	  A(2,1) = 0.000000e+000;	  A(2,2) = 1.000000e+000;	  A(2,3) = 2.000000e+000;	
    A(3,0) = 0.000000e+000;	  A(3,1) = 0.000000e+000;	  A(3,2) = 0.000000e+000;	  A(3,3) = 1.000000e+000;	

    y(0) = 0.000000e+000;	  y(1) = 1.942890e-016;	  y(2) = -2.775558e-016;	  y(3) = 1.000000e+000;	

    size_t R = 5;
    double eps = 1.000000e-004;
    size_t M = 5;
    x.clear();
    double relative_residual_error = 0.0;
    size_t used_inner_iterations = 0;
    size_t used_outer_iterations = 0;
    size_t status = 0;
    OpenTissue::math::big::gmres(A,x,b,M,R,eps,relative_residual_error,used_inner_iterations, used_outer_iterations,status);
    //for(size_type i = 0; i < x.size();++i)
    //  BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
    BOOST_CHECK(  used_inner_iterations == 4 );
    BOOST_CHECK(  used_outer_iterations == 1 );
    //BOOST_CHECK(  relative_residual_error < eps );
    BOOST_CHECK(  status == 0 );
  }
}

BOOST_AUTO_TEST_SUITE_END();
