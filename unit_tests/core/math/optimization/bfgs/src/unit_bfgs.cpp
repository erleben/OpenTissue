//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_bfgs.h>
#include <OpenTissue/core/math/big/big_generate_random.h>
#include <OpenTissue/core/math/big/big_generate_PD.h>
#include <OpenTissue/core/math/big/io/big_matlab_write.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>

typedef double real_type;
typedef ublas::compressed_matrix<real_type> matrix_type;
typedef ublas::vector<real_type>            vector_type;
typedef vector_type::size_type              size_type;

class F
{
public:
  matrix_type const & m_A;
  vector_type const & m_b;

  F(matrix_type const & A, vector_type const & b)
    : m_A(A)
    , m_b(b)
  {}

  real_type operator()( vector_type const & x ) const
  {
    return ublas::inner_prod(x, ublas::prod(m_A,x)) - inner_prod(m_b, x);
  }
};

class nabla_F
{
public:
  matrix_type const & m_A;
  vector_type const & m_b;

  nabla_F(matrix_type const & A, vector_type const & b)
    : m_A(A)
    , m_b(b)
  {}

  vector_type operator()( vector_type const & x ) const
  {
    return  vector_type( 2*ublas::prod(m_A,x) - m_b );
  }
};

template<typename func_functor, typename grad_functor>
void do_unconstrained_minimizer_test(func_functor & f, grad_functor & nabla_f, vector_type & x, matrix_type & H, vector_type const & solution )
{
  using namespace OpenTissue::math::big;

  size_type max_iterations       = 100;
  real_type absolute_tolerance   = boost::numeric_cast<real_type>(1e-6);
  real_type relative_tolerance   = boost::numeric_cast<real_type>(0.000000001);
  real_type stagnation_tolerance = boost::numeric_cast<real_type>(0.000000001);
  size_t status = 0;
  size_type iteration = 0;
  real_type accuracy = boost::numeric_cast<real_type>(0.0);
  real_type alpha = boost::numeric_cast<real_type>(0.0001);
  real_type beta = boost::numeric_cast<real_type>(0.5);

  OpenTissue::math::optimization::bfgs(
    f
    , nabla_f
    , H
    , x 
    , max_iterations
    , absolute_tolerance
    , relative_tolerance
    , stagnation_tolerance
    , status
    , iteration
    , accuracy
    , alpha
    , beta
    );

  std::cout << "status     = " 
    << OpenTissue::math::optimization::get_error_message(status) 
    << std::endl;
  std::cout << "absolute   = " 
    << accuracy  
    << std::endl;
  std::cout << "iterations = " 
    << iteration 
    << std::endl;
  std::cout << "x          = " 
    << x 
    << std::endl;

  if(status==OpenTissue::math::optimization::ABSOLUTE_CONVERGENCE)
  {
    BOOST_CHECK( accuracy < absolute_tolerance );
    BOOST_CHECK( iteration <= max_iterations );
  }

  double tol = 0.001;
  BOOST_CHECK_CLOSE( x(0), solution(0), tol);
  BOOST_CHECK_CLOSE( x(1), solution(1), tol);
}






class F_rosenbrock
{
public:

  F_rosenbrock(){}

  real_type operator()( vector_type const & x ) const
  {
    real_type x_1 = x(0);
    real_type x_2 = x(1);        
    return (10.0*(x_2-x_1*x_1)*(x_2-x_1*x_1) + (1- x_1)*(1- x_1));
  }
};



class nabla_F_rosenbrock
{
public:

  nabla_F_rosenbrock(){}

  vector_type operator()( vector_type const & x ) const
  {
    real_type x_1 = x(0);
    real_type x_2 = x(1);
    vector_type retur(2);
    retur(0)=-40.0*(x_1*x_2 -x_1*x_1*x_1) - 2*(1-x_1);
    retur(1)=20.0*(x_2-x_1*x_1);
    return  retur;
  }
};

BOOST_AUTO_TEST_SUITE(opentissue_math_big_bfgs);

BOOST_AUTO_TEST_CASE(simple_test_case)
{
  using namespace OpenTissue::math::big;

  // We are solving the problem
  //
  //   min_x Q(x) = x^T A x - b^T x
  //
  // where the gradient is given by
  //
  //   nabla Q(x) = 2 A x - b = 0
  //
  // and the exact Hessian is
  //
  //   H = nabla^2 Q(x) = 2 A
  //
  // The stationary points are given by 
  //
  //  | 4 0| |x_1| + | -1| = 0
  //  | 0 4| |x_2|   | -2|
  //
  // and has the unique solution x = [-0.25, -0.5]^T
  //
  size_type N = 2;

  matrix_type A;
  A.resize(N,N,false);

  vector_type b;
  b.resize(N,false);

  A(0,0) = 2.0;  A(0,1) = 0.0;
  A(1,0) = 0.0;  A(1,1) = 2.0;  

  b(0) = -1.0;
  b(1) = -2.0;

  vector_type solution;
  solution.resize(N,false);
  solution(0) = -0.25;
  solution(1) = -0.5;

  F f(A,b);
  nabla_F nabla_f(A,b);

  vector_type x;
  x.resize(N,false);
  matrix_type H;
  H.resize(N,N,false);

  // use H = I/4, and x = 0
  x.clear();
  H(0,0) = 0.25;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 0.25;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = I, and x = 0
  x.clear();
  H(0,0) = 1.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 1.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = 4*I, and x = 0
  x.clear();
  H(0,0) = 4.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 4.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = I/4, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  H(0,0) = 0.25;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 0.25;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = I, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  H(0,0) = 1.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 1.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = 4*I, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  H(0,0) = 4.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 4.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = random PD, and x = 0
  x.clear();
  OpenTissue::math::big::generate_PD(2, H);
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = random PD, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  OpenTissue::math::big::generate_PD(2, H);
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // H = g g^T, x = 0
  x.clear();
  vector_type g;
  g.resize(N,false);
  g = nabla_f(x);
  H = ublas::outer_prod(g,g);
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // H = g g^T, x = random
  OpenTissue::math::big::generate_random( 2, x);
  g = nabla_f(x);
  H = ublas::outer_prod(g,g);
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // H = exact Hessian, x = solution!
  x(0) = -0.25;
  x(1) = -0.5;
  H(0,0) = 4.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 4.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

}

BOOST_AUTO_TEST_CASE(rosenbrock_test_case)
{
  using namespace OpenTissue::math::big;

  size_type N = 2;

  vector_type solution;
  solution.resize(N,false);
  solution(0) =  1.0;
  solution(1) =  1.0;

  F_rosenbrock f;
  nabla_F_rosenbrock nabla_f;

  vector_type x;
  x.resize(N,false);
  matrix_type H;
  H.resize(N,N,false);

  // use H = I/4, and x = 0
  x.clear();
  x(0)=2.0;
  x(1)=2.0;
  H(0,0) = 0.25;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 0.25;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = I, and x = 0
  std::cout << std::endl;

  x.clear();
  x(0)=2.0;
  x(1)=2.0;
  std::cout << "using H = I   x = " << x <<  std::endl;
  H(0,0) = 1.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 1.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = 4*I, and x = 0
  std::cout << std::endl;

  x.clear();
  std::cout << "using H = 4*I   x = "<< x << std::endl;
  H(0,0) = 4.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 4.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = I/4, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  x *= 3.0;
  H(0,0) = 0.25;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 0.25;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = I, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  x *= 3.0;
  H(0,0) = 1.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 1.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = 4*I, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  x *= 3.0;
  H(0,0) = 4.0;   H(0,1) = 0.0;
  H(1,0) = 0.0;    H(1,1) = 4.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = random PD, and x = 0
  x.clear();
  OpenTissue::math::big::generate_PD(2, H);
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // use H = random PD, and x = random
  OpenTissue::math::big::generate_random( 2, x);
  x *= 3.0;
  OpenTissue::math::big::generate_PD(2, H);
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

  // H = exact Hessian, x = solution!
  x(0) = 1.0;
  x(1) = 1.0;
  H(0,0) = 82.0;   H(0,1) = -40.0;
  H(1,0) = -40.0;    H(1,1) = 20.0;   
  do_unconstrained_minimizer_test(f,nabla_f,x,H,solution);

}

BOOST_AUTO_TEST_SUITE_END();
