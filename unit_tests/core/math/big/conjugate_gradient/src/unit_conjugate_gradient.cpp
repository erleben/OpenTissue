//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_conjugate_gradient.h>


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

  real_type const tol = boost::numeric_cast<real_type>(1.0);

  solver_type S;

  real_type epsilon = boost::numeric_cast<real_type>(10e-10);
  size_t max_iterations = 100;
  size_t iterations;

  S(A,x,b, max_iterations, epsilon, iterations);

  // Test for convergence
  BOOST_CHECK( iterations < max_iterations );

  for(size_type i = 0; i < x.size();++i)
    BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
}



BOOST_AUTO_TEST_SUITE(opentissue_math_big_conjugate_gradient);


BOOST_AUTO_TEST_CASE(logic_and_valid_arguments_testing)
{
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef vector_type::size_type           size_type;

  typedef OpenTissue::math::big::ConjugateGradientFunctor solver_type;

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
  {
    matrix_type A;
    vector_type x,b;
    A.resize(4,5,false);
    x.resize(5,false);
    b.resize(4,false);
    solver_type S;
    BOOST_CHECK_THROW( S(A,x,b), std::invalid_argument );
  }
}





BOOST_AUTO_TEST_CASE(random_test_case)
{

  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef vector_type::size_type           size_type;

  typedef OpenTissue::math::big::ConjugateGradientFunctor solver_type;


  for(size_type tst=0;tst<1000;++tst)
  {
    size_type N = 10;

    matrix_type A;
    A.resize(N,N,false);
    
    vector_type x;
    x.resize(N,false);
    
    vector_type b;
    b.resize(N,false);

    vector_type y;
    y.resize(N,false);

    matrix_type R;
    R.resize(N,N,false);

    OpenTissue::math::Random<double> value(0.0,1.0);
    for(size_t i=0;i<R.size1();++i)
    { 
      b(i) = value();
      x(i) = value();
      y(i) = value();
      for(size_t j=0;j<R.size2();++j)
        R(i,j) = value();
    }
    ublas::noalias(A) = ublas::sparse_prod<matrix_type>( ublas::trans(R), R );
    // forcing A to become PD matrix (it should be non-singular at all times)
    for(size_t i=0;i<R.size1();++i)
      A(i,i) += 0.5;

    ublas::noalias(b) = ublas::prod(A,x);
    y.assign(x);

    x.clear();
    test<solver_type>(A,x,b,y);
  }

}

BOOST_AUTO_TEST_SUITE_END();
