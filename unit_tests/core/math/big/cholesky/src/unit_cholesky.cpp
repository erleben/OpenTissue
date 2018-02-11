//
// OpenTissue, A toolbox for physical based simulation and animation.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_cholesky.h>

#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/symmetric.hpp>

namespace ublas = boost::numeric::ublas;

#include <cassert>
#include <limits>


#define BOOST_AUTO_TEST_MAIN
#include <OpenTissue/utility/utility_push_boost_filter.h>
#include <boost/test/auto_unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/test_tools.hpp>
#include <OpenTissue/utility/utility_pop_boost_filter.h>


/** 
* Make a immutable triangular adaptor from a matrix
*
* \usage: 
<code>
A = triangular< lower >(B);
A = triangular(B, lower());
</code>
*/
template < class TYPE, class MATRIX >
ublas::triangular_adaptor<const MATRIX, TYPE>
triangular(const MATRIX & A, const TYPE& uplo = TYPE())
{
  return ublas::triangular_adaptor<const MATRIX, TYPE>(A);
}

/** 
* Make a immutable banded adaptor from a matrix
*
* \usage: 
<code>
A = banded(B, lower, upper);
</code>
*/
template < class MATRIX >
ublas::banded_adaptor<const MATRIX>
banded(const MATRIX & A, const size_t lower, const size_t upper)
{
  return ublas::banded_adaptor<const MATRIX>(A, lower, upper);
}

/** 
* Make a immutable symmetric adaptor from a matrix
*
* \usage: 
<code>
A = symmetric< lower >(B);
A = symmetric(B, lower());
</code>
*/
template < class TYPE, class MATRIX >
ublas::symmetric_adaptor<const MATRIX, TYPE>
symmetric(const MATRIX & A, const TYPE& uplo = TYPE())
{
  return ublas::symmetric_adaptor<const MATRIX, TYPE>(A);
}


/** 
* Fill lower triangular matrix L 
*/
template < class MATRIX >
void fill_symm(MATRIX & L, const size_t bands = std::numeric_limits<size_t>::max() )
{
  typedef typename MATRIX::size_type size_type;

  assert(L.size1() == L.size2());

  size_type size = L.size1();
  for (size_type i=0; i<size; i++) 
  {
    for (size_type j = ((i>bands)?(i-bands):0); j<i; j++) 
    {
      L(i,j) = 1 + (1.0 + i)/(1.0 + j) + 1.0 / (0.5 + i - j);
    }
    L(i,i) = 1+i+size;
  }

  return;
}




template<typename matrix_type,typename vector_type>
void test(matrix_type const & A, vector_type  & x, vector_type const & b, vector_type const & y)
{
  using std::min;

  typedef typename matrix_type::value_type real_type;
  typedef typename matrix_type::size_type  size_type;

  real_type const tol = boost::numeric_cast<real_type>(0.01);

  size_type const m = A.size1();
  size_type const n = A.size2();
  matrix_type L;
  L.resize(m,n);

  // Dense versions
  OpenTissue::math::big::cholesky_decompose(A,L);
  x = b;
  OpenTissue::math::big::cholesky_solve(L, x, ublas::lower());
  for(size_type i = 0; i < x.size();++i)
    BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
  x.clear();

  L.assign(A);
  OpenTissue::math::big::cholesky_decompose(L);
  x = b;
  OpenTissue::math::big::cholesky_solve(L, x, ublas::lower());
  for(size_type i = 0; i < x.size();++i)
    BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
  x.clear();

  // Easy Dense version
  L.assign(A);
  OpenTissue::math::big::cholesky_solve(L,x,b);
  for(size_type i = 0; i < x.size();++i)
    BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
  x.clear();

  // sparse version
  ublas::compressed_matrix<double> C;
  C.resize(m,n,false);
  C.assign(A);
  OpenTissue::math::big::incomplete_cholesky_decompose(C);
  x = b;
  OpenTissue::math::big::cholesky_solve(C, x, ublas::lower());
  for(size_type i = 0; i < x.size();++i)
    BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );
  x.clear();

  // Easy sparse version
  C.assign(A);
  OpenTissue::math::big::cholesky_solve(C,x,b);
  for(size_type i = 0; i < x.size();++i)
    BOOST_CHECK_CLOSE( real_type( x(i) ), real_type( y(i) ), tol );

}

BOOST_AUTO_TEST_SUITE(opentissue_math_big_cholesky);

BOOST_AUTO_TEST_CASE(Gunter_Winkler_Konstantin_Kutzkow_Test)
{
  size_t size = 10;
  {
    // use dense matrix
    ublas::matrix<double, ublas::row_major> A (size, size);
    ublas::matrix<double, ublas::row_major> T (size, size);
    ublas::matrix<double, ublas::row_major> L (size, size);

    A = ublas::zero_matrix<double>(size, size);

    ublas::vector<double> b (size);
    ublas::vector<double> x (size);
    ublas::vector<double> y (size);

    std::fill(y.begin(), y.end(), 1.0);

    fill_symm(T);
    A = ublas::prod(T, trans(T));
    b = prod( A, y);
    size_t res = OpenTissue::math::big::cholesky_decompose(A, L);
    
    BOOST_CHECK( res == 0u );
    
    x = b;
    OpenTissue::math::big::cholesky_solve(L, x, ublas::lower());
    //std::cout << res << ": " 
    //          << ublas::norm_inf(L-T) << " "
    //          << ublas::norm_2(x-y) << " "
    //          << " (deco: " << watch1() << " sec)"
    //          << " (prod: " << watch2() << " sec)"
    //          << " (solve: " << watch3() << " sec)"
    //          << " " << size
    //          << std::endl;
  }

  {
    // use dense triangular matrices
    ublas::triangular_matrix<double, ublas::lower, ublas::row_major> A (size, size);
    ublas::triangular_matrix<double, ublas::lower, ublas::row_major> T (size, size);
    ublas::triangular_matrix<double, ublas::lower, ublas::row_major> L (size, size);

    A = ublas::zero_matrix<double> (size, size) ;
    A = triangular<ublas::lower>( ublas::zero_matrix<double> (size, size) );
    A = triangular( ublas::zero_matrix<double> (size, size), ublas::lower() );

    ublas::vector<double> b (size);
    ublas::vector<double> x (size);
    ublas::vector<double> y (size);

    std::fill(y.begin(), y.end(), 1.0);

    fill_symm(T);
    A = triangular<ublas::lower>( ublas::prod(T, trans(T)) );
    b = prod( symmetric<ublas::lower>(A), y);
    size_t res = OpenTissue::math::big::cholesky_decompose(A, L);
    
    BOOST_CHECK( res == 0u );

    x = b;
    OpenTissue::math::big::cholesky_solve(L, x, ublas::lower());
    //std::cout << res << ": " 
    //          << ublas::norm_inf(L-T) << " "
    //          << ublas::norm_2(x-y) << " "
    //          << " (deco: " << watch1() << " sec)"
    //          << " (prod: " << watch2() << " sec)"
    //          << " (solve: " << watch3() << " sec)"
    //          << " " << size
    //          << std::endl;
  }
  {
    // use banded matrices matrix
    typedef ublas::banded_matrix<double, ublas::row_major> MAT;

    size_t bands = std::min<size_t>( size, 50 );
    MAT A (size, size, bands, 0);
    MAT T (size, size, bands, 0);
    MAT L (size, size, bands, 0);

    A = ublas::zero_matrix<double> (size, size) ;
    A = banded( ublas::zero_matrix<double> (size, size), bands, 0 );

    ublas::vector<double> b (size);
    ublas::vector<double> x (size);
    ublas::vector<double> y (size);

    std::fill(y.begin(), y.end(), 1.0);

    fill_symm(T, bands);
    A = banded( ublas::prod(T, trans(T)), bands, 0 );
    b = prod( symmetric<ublas::lower>(A), y);
    size_t res = OpenTissue::math::big::cholesky_decompose(A, L);
    
    BOOST_CHECK( res == 0u );
   
    x = b;
    OpenTissue::math::big::cholesky_solve(L, x, ublas::lower());
    //std::cout << res << ": " 
    //          << ublas::norm_inf(L-T) << " "
    //          << ublas::norm_2(x-y) << " "
    //          << " (deco: " << watch1() << " sec)"
    //          << " (prod: " << watch2() << " sec)"
    //          << " (solve: " << watch3() << " sec)"
    //          << " " << size
    //          << std::endl;
  }
}

BOOST_AUTO_TEST_CASE(random_test_case)
{
  typedef ublas::matrix<double>            matrix_type;
  typedef ublas::vector<double>            vector_type;
  typedef vector_type::size_type           size_type;

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
    ublas::noalias(A) = ublas::prod( ublas::trans(R), R );
    // forcing A to become PD matrix (it should be non-singular at all times)
    for(size_t i=0;i<R.size1();++i)
      A(i,i) += 0.5;

    ublas::noalias(b) = ublas::prod(A,x);
    y.assign(x);

    x.clear();
    test(A,x,b,y);
  }

}

BOOST_AUTO_TEST_SUITE_END();
