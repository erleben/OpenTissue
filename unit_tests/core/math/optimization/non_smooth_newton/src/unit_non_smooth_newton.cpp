//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_generate_PD.h>
#include <OpenTissue/core/math/big/big_generate_PSD.h>
#include <OpenTissue/core/math/big/big_generate_random.h>
#include <OpenTissue/core/math/optimization/optimization_non_smooth_newton.h>
#include <OpenTissue/core/math/big/big_gmres.h>
#include <OpenTissue/core/math/big/big_svd.h>

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
    
    class vector_iterator
    {
    protected:
      
      bool m_end;
      size_type m_idx;
      real_type m_mu;
      
    public:
      
      vector_iterator()
      : m_end(true)
      , m_idx(0)
      , m_mu(value_traits::zero())
      {}
      
      vector_iterator(vector_iterator const & i)
      {
        *this = i;
      }
      
      vector_iterator(size_type const & idx, real_type const & mu)
      : m_end(false)
      , m_idx(idx)
      , m_mu(mu)
      {}
      
      bool const operator==(vector_iterator const & i) const     {      return this->m_end == i.m_end;     }
      bool const operator!=(vector_iterator const & i) const {  return !( (*this) == i); }
      
      size_t const index() const { return m_idx; }
      
      real_type operator*() const { return m_mu; }
      
      vector_iterator & operator=(vector_iterator const & i) 
      {
        this->m_end = i.m_end;
        this->m_idx = i.m_idx;
        this->m_mu = i.m_mu;
        return *this;
      }
      
      
      vector_iterator const & operator++() 
      {
        m_end = true;
        return *this; 
      }
    };
    
    
    vector_iterator partial_begin(size_type const & idx) const 
    {
      size_type r = (idx%3);
      
      if(r==0)
        return vector_iterator();
      
      real_type mu_i = value_traits::one();
      size_type j = idx - r;
      return m_is_lower ? vector_iterator(j,-mu_i) :  vector_iterator(j, mu_i);
    }
    
    vector_iterator partial_end(size_type const & idx) const {  return vector_iterator(); }
    
  };



template<typename matrix_type,typename vector_type>
void test(matrix_type const & A, vector_type  & x, vector_type const & b, vector_type const & y, size_t * cnt_status)
{
  typedef typename matrix_type::value_type real_type;
  typedef typename matrix_type::size_type  size_type;
  
  BoundFunction<vector_type> l(true);
  BoundFunction<vector_type> u(false);
  
  size_type max_iterations       = 100;
  real_type absolute_tolerance   = boost::numeric_cast<real_type>(1e-6);
  real_type relative_tolerance   = boost::numeric_cast<real_type>(0.000000001);
  real_type stagnation_tolerance = boost::numeric_cast<real_type>(0.000000001);
  size_t status = 0;
  size_type iteration = 0;
  real_type accuracy = boost::numeric_cast<real_type>(0.0);
  real_type alpha = boost::numeric_cast<real_type>(0.0001);
  real_type beta = boost::numeric_cast<real_type>(0.5);
  bool use_shur = true;
  

  OpenTissue::math::optimization::non_smooth_newton( 
                                                    A, b, l , u, x 
                                                    , max_iterations
                                                    , absolute_tolerance
                                                    , relative_tolerance
                                                    , stagnation_tolerance
                                                    , status
                                                    , iteration
                                                    , accuracy
                                                    , alpha
                                                    , beta
                                                    , &OpenTissue::math::big::svd<matrix_type, vector_type>
                                                    , use_shur
                                                    );
  
  cnt_status[status]++;
  
  if(status==OpenTissue::math::optimization::ABSOLUTE_CONVERGENCE)
  {
    BOOST_CHECK( accuracy < absolute_tolerance );
    BOOST_CHECK( iteration <= max_iterations );
  }
  std::cout << "absolute " << accuracy << " iterations " << iteration  << " status " << OpenTissue::math::optimization::get_error_message(status) << std::endl;
}

BOOST_AUTO_TEST_SUITE(opentissue_math_big_non_smooth_newton);

BOOST_AUTO_TEST_CASE(random_test_case)
{
  
  typedef ublas::compressed_matrix<double> matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef vector_type::size_type           size_type;
  
  size_t cnt_status[7] = {0, 0, 0, 0, 0, 0, 0};
  
  size_type N = 10;
  
  matrix_type A;
  vector_type x;
  vector_type b;
  vector_type y;
  
  A.resize(N,N,false);
  x.resize(N,false);
  b.resize(N,false);
  y.resize(N,false);
  
  for(size_type tst=0;tst<100;++tst)
  {
    OpenTissue::math::big::fast_generate_PD(N,A);
    OpenTissue::math::big::generate_random( N, y);
    b.assign(-y);
    OpenTissue::math::big::generate_random( N, y);
    x.clear();
    
    test(A,x,b,y, &cnt_status[0]);
  }
  
  for(size_t i=0;i<7;++i)
    std::cout << cnt_status[i] << "\t:\t"<< OpenTissue::math::optimization::get_error_message( i ) << std::endl;
  std::cout << std::endl;
}

BOOST_AUTO_TEST_SUITE_END();
