//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/optimization/optimization_projected_gauss_seidel.h>

#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


template <typename vector_type>
class BoundFunction
{
public:

  typedef typename vector_type::size_type          size_type;
  typedef typename vector_type::value_type         real_type;
  typedef OpenTissue::math::ValueTraits<real_type> value_traits;


  bool m_is_lower;

public:

  BoundFunction(bool const & is_lower)
    : m_is_lower(is_lower)
  {}

  real_type operator()(vector_type const & x, size_type const & i) const
  {
    size_type r = (i%3);

    if(r==0)
      return m_is_lower ? value_traits::zero() : value_traits::infinity();

    real_type mu_i = value_traits::one();
    size_type j = i - r;
    return m_is_lower ?  -mu_i*x(j) : mu_i*x(j);
  }

};

template<typename matrix_type,typename vector_type>
void test(matrix_type const & A, vector_type  & x, vector_type const & b, vector_type const & y)
{
  typedef typename matrix_type::value_type real_type;
  typedef typename matrix_type::size_type  size_type;

  BoundFunction<vector_type> l(true);
  BoundFunction<vector_type> u(false);

  size_type max_iterations       = 1000;
  real_type absolute_tolerance   = boost::numeric_cast<real_type>(0.025);
  real_type relative_tolerance   = boost::numeric_cast<real_type>(0.00001);
  real_type stagnation_tolerance = boost::numeric_cast<real_type>(0.00001);
  size_t status = 0;
  size_type iteration = 0;
  real_type accuracy = boost::numeric_cast<real_type>(0.0);
  real_type relative_accuracy = boost::numeric_cast<real_type>(0.0);

  OpenTissue::math::optimization::projected_gauss_seidel( 
    A, b, l , u, x 
    ,  max_iterations
    ,  absolute_tolerance
    ,  relative_tolerance
    ,  stagnation_tolerance
    ,  status
    ,  iteration
    ,  accuracy
    ,  relative_accuracy
    );

  if(status==OpenTissue::math::optimization::ABSOLUTE_CONVERGENCE)
  {
    BOOST_CHECK( accuracy < absolute_tolerance );
    BOOST_CHECK( iteration <= max_iterations );
  }
  else
  {
    std::cout << std::endl;
    std::cout << "absolute " << accuracy << " iter " << iteration << " status " << status << " relative " << relative_accuracy << std::endl;
    std::cout << std::endl;
  }
}

BOOST_AUTO_TEST_SUITE(opentissue_math_big_projected_gauss_seidel);

BOOST_AUTO_TEST_CASE(random_test_case)
{

  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef vector_type::size_type           size_type;

  for(size_type tst=0;tst<1000;++tst)
  {
    size_type N = 20;

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
      b(i) = -value();

      x(i) = value();
      y(i) = value();
      for(size_t j=0;j<R.size2();++j)
        R(i,j) = value();
    }
    ublas::noalias(A) = ublas::sparse_prod<matrix_type>( ublas::trans(R), R );
    // forcing A to become PD matrix (it should be non-singular at all times)
    for(size_t i=0;i<R.size1();++i)
      A(i,i) += 0.5;

    x.clear();
    test(A,x,b,y);
  }

}

BOOST_AUTO_TEST_SUITE_END();
