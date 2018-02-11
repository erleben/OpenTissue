//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_jacobi.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

template<typename solver_type, typename matrix_type,typename vector_type>
void test(matrix_type const & A, vector_type  & x, vector_type const & b, vector_type const & y)
{
  typedef typename matrix_type::value_type real_type;
  typedef typename matrix_type::size_type  size_type;

  real_type const tol = boost::numeric_cast<real_type>(0.01);

  solver_type S;

  S(A,x,b);

  for(size_type i = 0; i < x.size();++i)
    BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
}


BOOST_AUTO_TEST_SUITE(opentissue_math_big_jacobi);

BOOST_AUTO_TEST_CASE(logic_and_valid_arguments_testing)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;

  typedef OpenTissue::math::big::JacobiFunctor solver_type;

  {
    matrix_type A;
    vector_type x,b;
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,4,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,4,false);
    b.resize(4,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,4,false);
    x.resize(4,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,4,false);
    x.resize(4,false);
    b.resize(4,false);
    A(0,0) = A(1,1) = A(2,2) = A(3,3) = 1.0;
    solver_type S;
    BOOST_CHECK_NO_THROW( S(A,x,b) );
  }
  {
    matrix_type A;
    vector_type x,b;
    x.resize(4,false);
    b.resize(4,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,4,false);
    x.resize(3,false);
    b.resize(4,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,4,false);
    x.resize(4,false);
    b.resize(3,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,4,false);
    x.resize(3,false);
    b.resize(3,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
}


BOOST_AUTO_TEST_CASE(correctness_testing)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;

  typedef OpenTissue::math::big::JacobiFunctor solver_type;

  matrix_type A;
  A.resize(4,4,false);
  vector_type x,b,y;
  x.resize(4,false);
  b.resize(4,false);
  y.resize(4,false);


  x(0) = 1.000000;
  x(1) = 2.000000;
  x(2) = 3.000000;
  x(3) = 4.000000;
  b(0) = 4.000000;
  b(1) = 3.000000;
  b(2) = 2.000000;
  b(3) = 1.000000;
  y(0) = -25.000000;
  y(1) = -9.166667;
  y(2) = -6.818182;
  y(3) = -5.312500;
  A(0,0) = 1.000000;
  A(0,1) = 2.000000;
  A(0,2) = 3.000000;
  A(0,3) = 4.000000;
  A(1,0) = 5.000000;
  A(1,1) = 6.000000;
  A(1,2) = 7.000000;
  A(1,3) = 8.000000;
  A(2,0) = 9.000000;
  A(2,1) = 10.000000;
  A(2,2) = 11.000000;
  A(2,3) = 12.000000;
  A(3,0) = 13.000000;
  A(3,1) = 14.000000;
  A(3,2) = 15.000000;
  A(3,3) = 16.000000;
  test<solver_type>(A,x,b,y);

  x(0) = 0.950129;
  x(1) = 0.231139;
  x(2) = 0.606843;
  x(3) = 0.485982;
  b(0) = 0.057891;
  b(1) = 0.352868;
  b(2) = 0.813166;
  b(3) = 0.009861;
  y(0) = -1.285747;
  y(1) = -2.844137;
  y(2) = 0.214613;
  y(3) = -0.488969;
  A(0,0) = 0.891299;
  A(0,1) = 0.821407;
  A(0,2) = 0.921813;
  A(0,3) = 0.935470;
  A(1,0) = 0.762097;
  A(1,1) = 0.444703;
  A(1,2) = 0.738207;
  A(1,3) = 0.916904;
  A(2,0) = 0.456468;
  A(2,1) = 0.615432;
  A(2,2) = 0.176266;
  A(2,3) = 0.410270;
  A(3,0) = 0.018504;
  A(3,1) = 0.791937;
  A(3,2) = 0.405706;
  A(3,3) = 0.893650;
  test<solver_type>(A,x,b,y);

  x(0) = 0.138891;
  x(1) = 0.202765;
  x(2) = 0.198722;
  x(3) = 0.603792;
  b(0) = 0.831796;
  b(1) = 0.502813;
  b(2) = 0.709471;
  b(3) = 0.428892;
  y(0) = 3.055962;
  y(1) = 0.539606;
  y(2) = 3.501015;
  y(3) = 1.130208;
  A(0,0) = 0.272188;
  A(0,1) = 0.000000;
  A(0,2) = 0.000000;
  A(0,3) = 0.000000;
  A(1,0) = 0.000000;
  A(1,1) = 0.931815;
  A(1,2) = 0.000000;
  A(1,3) = 0.000000;
  A(2,0) = 0.000000;
  A(2,1) = 0.000000;
  A(2,2) = 0.202647;
  A(2,3) = 0.000000;
  A(3,0) = 0.000000;
  A(3,1) = 0.000000;
  A(3,2) = 0.000000;
  A(3,3) = 0.379481;
  test<solver_type>(A,x,b,y);

  //x(0) = 0.304617;
  //x(1) = 0.189654;
  //x(2) = 0.193431;
  //x(3) = 0.682223;
  //b(0) = 0.341194;
  //b(1) = 0.534079;
  //b(2) = 0.727113;
  //b(3) = 0.309290;
  ////y(0) = NaN;
  ////y(1) = NaN;
  ////y(2) = NaN;
  ////y(3) = NaN;
  //A(0,0) = 0.000000;
  //A(0,1) = 0.000000;
  //A(0,2) = 0.000000;
  //A(0,3) = 0.000000;
  //A(1,0) = 0.541674;
  //A(1,1) = 0.000000;
  //A(1,2) = 0.000000;
  //A(1,3) = 0.000000;
  //A(2,0) = 0.150873;
  //A(2,1) = 0.853655;
  //A(2,2) = 0.000000;
  //A(2,3) = 0.000000;
  //A(3,0) = 0.697898;
  //A(3,1) = 0.593563;
  //A(3,2) = 0.644910;
  //A(3,3) = 0.000000;
  //test<solver_type>(A,x,b,y);

  //x(0) = 0.838496;
  //x(1) = 0.568072;
  //x(2) = 0.370414;
  //x(3) = 0.702740;
  //b(0) = 0.893898;
  //b(1) = 0.199138;
  //b(2) = 0.298723;
  //b(3) = 0.661443;
  ////y(0) = NaN;
  ////y(1) = NaN;
  ////y(2) = NaN;
  ////y(3) = NaN;
  //A(0,0) = 0.000000;
  //A(0,1) = 0.794821;
  //A(0,2) = 0.172956;
  //A(0,3) = 0.875742;
  //A(1,0) = 0.000000;
  //A(1,1) = 0.000000;
  //A(1,2) = 0.979747;
  //A(1,3) = 0.737306;
  //A(2,0) = 0.000000;
  //A(2,1) = 0.000000;
  //A(2,2) = 0.000000;
  //A(2,3) = 0.136519;
  //A(3,0) = 0.000000;
  //A(3,1) = 0.000000;
  //A(3,2) = 0.000000;
  //A(3,3) = 0.000000;
  //test<solver_type>(A,x,b,y);

  x(0) = 0.284409;
  x(1) = 0.469224;
  x(2) = 0.064781;
  x(3) = 0.988335;
  b(0) = 0.794211;
  b(1) = 0.059183;
  b(2) = 0.602869;
  b(3) = 0.050269;
  y(0) = 1.362769;
  y(1) = -0.271137;
  y(2) = 0.881017;
  y(3) = -0.750396;
  A(0,0) = 0.582792;
  A(0,1) = 0.000000;
  A(0,2) = 0.000000;
  A(0,3) = 0.000000;
  A(1,0) = 0.423496;
  A(1,1) = 0.225950;
  A(1,2) = 0.000000;
  A(1,3) = 0.000000;
  A(2,0) = 0.515512;
  A(2,1) = 0.579807;
  A(2,2) = 0.209069;
  A(2,3) = 0.000000;
  A(3,0) = 0.333951;
  A(3,1) = 0.760365;
  A(3,2) = 0.379818;
  A(3,3) = 0.567829;
  test<solver_type>(A,x,b,y);

  x(0) = 0.415375;
  x(1) = 0.304999;
  x(2) = 0.874367;
  x(3) = 0.015009;
  b(0) = 0.683332;
  b(1) = 0.212560;
  b(2) = 0.839238;
  b(3) = 0.628785;
  y(0) = 0.336660;
  y(1) = -1.266157;
  y(2) = 1.145883;
  y(3) = 0.673664;
  A(0,0) = 0.767950;
  A(0,1) = 0.438659;
  A(0,2) = 0.320036;
  A(0,3) = 0.744566;
  A(1,0) = 0.000000;
  A(1,1) = 0.498311;
  A(1,2) = 0.960099;
  A(1,3) = 0.267947;
  A(2,0) = 0.000000;
  A(2,1) = 0.000000;
  A(2,2) = 0.726632;
  A(2,3) = 0.439924;
  A(3,0) = 0.000000;
  A(3,1) = 0.000000;
  A(3,2) = 0.000000;
  A(3,3) = 0.933380;
  test<solver_type>(A,x,b,y);



}


BOOST_AUTO_TEST_SUITE_END();
