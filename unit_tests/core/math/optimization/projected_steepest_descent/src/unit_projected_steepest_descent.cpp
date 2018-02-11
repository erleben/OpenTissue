//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_projected_steepest_descent.h>
#include <OpenTissue/core/math/big/big_generate_random.h>
#include <OpenTissue/core/math/big/big_generate_PD.h>
#include <OpenTissue/core/math/big/io/big_matlab_write.h>
#include <OpenTissue/core/math/optimization/optimization_project.h>

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

class ProjectionOperator
{
public:
  vector_type const & m_l;
  vector_type const & m_u;

  ProjectionOperator(vector_type const & l, vector_type const & u)
    : m_l(l)
    , m_u(u)
  {}

  vector_type operator()( vector_type const & x ) const
  {
    vector_type y;
    y.resize(x.size());
    OpenTissue::math::optimization::project(x,m_l,m_u,y);
    return  y;
  }
};




void do_test(F & f, nabla_F & nabla_f, vector_type & x, ProjectionOperator & P, vector_type const & y)
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

  OpenTissue::math::optimization::projected_steepest_descent(
    f
    , nabla_f
    , x 
    , P
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
  for(size_t i = 0;i<x.size();++i)
    BOOST_CHECK_CLOSE( x(i), y(i), tol);
}

BOOST_AUTO_TEST_SUITE(opentissue_math_big_projected_steepest_descent);

BOOST_AUTO_TEST_CASE(unconstrained_global_minimizer)
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

  F f(A,b);
  nabla_F nabla_f(A,b);

  vector_type x;
  x.resize(N,false);

  vector_type y;
  y.resize(N,false);
  y(0) = -0.25;
  y(1) = -0.5;

  vector_type l;
  vector_type u;

  l.resize(N,false);
  u.resize(N,false);

  l(0) = -1.0;
  l(1) = -1.0;
  u(0) =  1.0;
  u(1) =  1.0;

  ProjectionOperator P(l,u);

  // use x = 0
  x.clear();
  do_test(f,nabla_f,x,P,y);

  // use x = random
  OpenTissue::math::big::generate_random( 2, x);
  do_test(f,nabla_f,x,P,y);

  // Use x = solution!
  x(0) = -0.25;
  x(1) = -0.5;
  do_test(f,nabla_f,x,P,y);
}


BOOST_AUTO_TEST_CASE(constrained_global_minimizer)
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

  F f(A,b);
  nabla_F nabla_f(A,b);

  vector_type x;
  x.resize(N,false);

  vector_type l;
  vector_type u;
  l.resize(N,false);
  u.resize(N,false);
  l(0) = 0.0;
  l(1) = -1.0;
  u(0) =  1.0;
  u(1) =  1.0;

  vector_type y;
  y.resize(N,false);
  y(0) = 0.0;
  y(1) = -0.5;

  ProjectionOperator P(l,u);

  // use  x = 0
  x.clear();
  do_test(f,nabla_f,x,P,y);

  // use  x = random
  OpenTissue::math::big::generate_random( 2, x);
  do_test(f,nabla_f,x,P,y);

  // use x = solution!
  x(0) = -0.25;
  x(1) = -0.5;
  do_test(f,nabla_f,x,P,y);

}

BOOST_AUTO_TEST_SUITE_END();
