#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_LU_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_LU_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#ifdef USE_ATLAS
#  include <boost/numeric/bindings/atlas/cblas2.hpp>
#  include <boost/numeric/bindings/atlas/cblas3.hpp>
#  include <boost/numeric/bindings/atlas/clapack.hpp>
#  include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#  include <boost/numeric/bindings/traits/ublas_vector.hpp>

namespace atlas = boost::numeric::bindings::atlas;
#endif

#include <OpenTissue/core/math/big/big_types.h>  

#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include <stdexcept>


namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

#ifdef USE_ATLAS

      /**
      * Solve Linear System using LU decomposition.
      *
      * @param A       The matrix.
      * @param x       Upon return this argument holds the solution 
      * @param b       The right hand side vector.
      *
      * @return      If succesfull then the return value is true otherwise it is false.
      */
      template<typename matrix_type, typename vector_type>
      inline bool lu_atlas( matrix_type  const & A, vector_type & x, vector_type const & b)
      {
        typedef typename matrix_type::value_type value_type;
        typedef typename matrix_type::size_type  size_type;

        if(A.size1() <= 0 || A.size2() <= 0)
          throw std::invalid_argument("A was empty");

        if(b.size() != A.size1())
          throw std::invalid_argument("The size of b must be the same as the number of rows in A");

        if(x.size() != A.size2())
          throw std::invalid_argument("The size of x must be the same as the number of columns in A");

        size_type m = A.size1();
        size_type n = A.size2();

        ublas::matrix<value_type, ublas::column_major> Acpy( m, n );
        Acpy.assign(A);
        ublas::matrix<value_type, ublas::column_major> B( n, 1 );
        ublas::column( B, 0 ) = b;
        atlas::gesv( Acpy, B );
        x = ublas::column( B, 0 );

        return true;
      }


      /**
      * Invert Matrix using LU factorization.
      *
      * @param A     The matrix to be inverted.
      * @param invA  Upon return this argument holds the inverted matrix.
      *
      * @note        This function works, and can be used as an
      *              alternative for big::lu_invert(). No performance measurements
      *              has been done to see which version is the fastest. However, this
      *              version uses ATLAS whereas the big::lu_invert version only uses
      *              ublas. 
      *
      * @return      If succesfull then the return value is true otherwise it is false.
      */
      template<typename T>
      inline bool lu_atlas_invert( ublas::matrix<T>  const & A, ublas::matrix<T> & invA)
      {
        typedef typename ublas::matrix<T>::size_type  size_type;

        if(A.size1() <= 0 || A.size2() <= 0)
          throw std::invalid_argument("A was empty");

        size_type m = A.size1();
        size_type n = A.size2();

        invA.resize(m,n,false);
        invA.assign( A );

        std::vector<int> ipiv (n);   // pivot vector
        int rc = atlas::lu_factor (invA, ipiv);  // alias for getrf()

        // FRom http://www.netlib.org/lapack/single/sgetrf.f
        //
        // INFO    (output) INTEGER
        //          = 0:  successful exit
        //          < 0:  if INFO = -i, the i-th argument had an illegal value
        //          > 0:  if INFO = i, U(i,i) is exactly zero. The factorization
        //                has been completed, but the factor U is exactly
        //                singular, and division by zero will occur if it is used
        //                to solve a system of equations.
        //
        if(rc!=0)
          return false;

        atlas::lu_invert (invA, ipiv);  // alias for getri()
        return true;
      }

#else

      /**
      * Solve Linear System using LU decomposition.
      *
      * @param A       The matrix.
      * @param x       Upon return this argument holds the solution 
      * @param b       The right hand side vector.
      *
      * @return      If succesfull then the return value is true otherwise it is false.
      */
      template<class matrix_type, class vector_type>
      inline bool lu(matrix_type const & A, vector_type & x, vector_type const & b) 
      {
        typedef typename matrix_type::size_type  size_type;
        typedef typename matrix_type::value_type real_type;
        typedef typename boost::numeric::ublas::permutation_matrix<size_t>		pmatrix_type;
        typedef typename boost::numeric::ublas::identity_matrix<real_type>	  identity_matrix_type;

        if(A.size1() <= 0 || A.size2() <= 0)
          throw std::invalid_argument("A was empty");

        if(b.size() != A.size1())
          throw std::invalid_argument("The size of b must be the same as the number of rows in A");

        if(x.size() != A.size2())
          throw std::invalid_argument("The size of x must be the same as the number of columns in A");

        matrix_type tmp(A);
        pmatrix_type pm(tmp.size1());
        int res = lu_factorize(tmp,pm);
        if( res != 0 ) 
          return false;
        x.assign(b);
        lu_substitute(tmp, pm, x);
        return true;
      }

      /**
      * LU matrix inversion.
      * This is a modified version of 
      *
      *  http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?action=browse&diff=1&id=LU_Matrix_Inversion
      *
      * @param A     The matrix to be inverted.
      * @param invA  Upon return this argument holds the inverted matrix.
      *
      * @return      If succesfully inverted then true otherwise false.
      */
      template<class matrix_type>
      inline bool lu_invert(matrix_type const & A, matrix_type& invA) 
      {
        typedef typename matrix_type::size_type  size_type;
        typedef typename matrix_type::value_type real_type;
        typedef typename boost::numeric::ublas::permutation_matrix<size_t>		pmatrix_type;
        typedef typename boost::numeric::ublas::identity_matrix<real_type>	  identity_matrix_type;

        if(A.size1() <= 0 || A.size2() <= 0)
          throw std::invalid_argument("A was empty");

        size_type m = A.size1();
        size_type n = A.size2();
        invA.resize(m,n,false);

        matrix_type tmp(A);

        pmatrix_type pm(tmp.size1());
        int res = lu_factorize(tmp,pm);
        if( res != 0 ) 
          return false;
        invA.assign(identity_matrix_type(tmp.size1()));
        lu_substitute(tmp, pm, invA);
        return true;
      }

#endif




    } // namespace big
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_LU_H
#endif
