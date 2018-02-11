#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_CHOLESKY_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_CHOLESKY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

//#include <OpenTissue/core/math/big/big_types.h> // Not needed for here! 

//
// 2007-9-28: The implementation in this file is based on a
// modification of code originally developed by Gunter Winkler and Konstantin Kutzkow.
// 
/** -*- c++ -*- \file cholesky.hpp \brief cholesky decomposition */
/*
-   begin                : 2005-08-24
-   copyright            : (C) 2005 by Gunter Winkler, Konstantin Kutzkow
-   email                : guwi17@gmx.de

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>

#include <boost/numeric/ublas/triangular.hpp>

namespace ublas = boost::numeric::ublas;
#include <cassert>
#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /** 
      * Decompose the symmetric positive definit matrix A into product L L^T.
      *
      * @param A        Square symmetric positive definite input matrix (only the lower triangle is accessed).
      * @param L        Upon return this argument holds the lower triangular output matrix. 
      * @return nonzero if decompositon fails (the value is 1 + the numer of the failing row)
      */
      template < typename matrix_type, typename triangular_matrix_type >
      inline size_t cholesky_decompose(matrix_type const & A, triangular_matrix_type& L)
      {
        using std::sqrt;
        using namespace ublas;

        typedef typename matrix_type::value_type T;

        if( A.size1() != A.size2() )
          throw std::invalid_argument("A could not be a symmetric matrix.");
        if( A.size1() != L.size1() )
          throw std::invalid_argument("Incompatible dimensions of A and L");
        if( A.size2() != L.size2() )
          throw std::invalid_argument("Incompatible dimensions of A and L");

        size_t const n = A.size1();

        for (size_t k = 0 ; k < n; ++k) 
        {        
          double qL_kk = A(k,k) - inner_prod( project( row(L, k), range(0, k) ), project( row(L, k), range(0, k) ) );
          if (qL_kk <= 0) 
          {
            return 1 + k;
          }
          else 
          {
            double L_kk = sqrt( qL_kk );
            L(k,k) = L_kk;

            matrix_column<triangular_matrix_type> cLk(L, k);

            project( cLk, range(k+1, n) )
              = ( project( column(A, k), range(k+1, n) )
              - prod( project(L, range(k+1, n), range(0, k)), 
              project(row(L, k), range(0, k) ) ) ) / L_kk;
          }
        }
        return 0;      
      }

      /** 
      * Decompose the symmetric positive definit matrix A into product L L^T.
      *
      * @param A   Upon call this argument holds a square symmetric positive definite matrix (only the lower triangle is accessed). Upon return the lower triangle of A is replaced by the cholesky factor
      * @return    nonzero if decompositon fails (the value is 1 + the numer of the failing row)
      */
      template < typename matrix_type >
      inline size_t cholesky_decompose(matrix_type & A)
      {
        using namespace ublas;
        using std::sqrt;

        typedef typename matrix_type::value_type T;

        matrix_type const & A_c(A);
        size_t const n = A.size1();

        for (size_t k=0 ; k < n; ++k) 
        {
          double qL_kk = A_c(k,k) - inner_prod( project( row(A_c, k), range(0, k) ), project( row(A_c, k), range(0, k) ) );
          if (qL_kk <= 0) 
          {
            return 1 + k;
          }
          else 
          {
            double L_kk = sqrt( qL_kk );

            matrix_column<matrix_type> cLk(A, k);

            project( cLk, range(k+1, n) )
              = ( project( column(A_c, k), range(k+1, n) )
              - prod( project(A_c, range(k+1, n), range(0, k)), 
              project(row(A_c, k), range(0, k) ) ) ) / L_kk;
            A(k,k) = L_kk;
          }
        }
        return 0;      
      }

      /** 
      * Decompose the symmetric positive definit matrix A into product L L^T.
      *
      * @param A    Upon invokation this argument holds a square symmetric positive definite matrix (only the lower triangle is accessed).Upon return the lower triangle of A is replaced by the cholesky factor.
      * @return     nonzero if decompositon fails (the value is 1 + the numer of the failing row)
      */
      template < typename matrix_type >
      inline size_t incomplete_cholesky_decompose(matrix_type & A)
      {
        using namespace ublas;
        using std::sqrt;

        typedef typename matrix_type::value_type T;

        // read access to a const matrix is faster
        matrix_type const & A_c(A);
        size_t const n = A.size1();

        for (size_t k=0 ; k < n; ++k)
        {
          double qL_kk = A_c(k,k) - inner_prod( project( row( A_c, k ), range(0, k) ), project( row( A_c, k ), range(0, k) ) );

          if (qL_kk <= 0) 
          {
            return 1 + k;
          }
          else 
          {
            double L_kk = sqrt( qL_kk );

            for (size_t i = k+1; i < A.size1(); ++i) 
            {
              T* Aik = A.find_element(i, k);

              if (Aik != 0) 
              {
                *Aik = ( *Aik - inner_prod( project( row( A_c, k ), range(0, k) ), project( row( A_c, i ), range(0, k) ) ) ) / L_kk;
              }
            }

            A(k,k) = L_kk;
          }
        }

        return 0;
      }

      /** 
      * Solve system L L^T x = b inplace
      *
      * @param L    A triangular matrix.
      * @param x    On invokation this argument holds the right hand side, upon return it holds the solution x.
      */
      template < typename triangular_matrix_type, typename vector_type >
      inline void cholesky_solve(triangular_matrix_type const & L, vector_type & x, ublas::lower)
      {
        using namespace ublas;
        inplace_solve(L, x, lower_tag() );
        inplace_solve(trans(L), x, upper_tag());
      }

      /**
      * Solve Linear System A x = b using Cholesky Factorization.
      *
      * @param A   The matrix (must be postive definite),
      * @param x   Upon return this argument contains the solution.
      * @param b   The right hand side vector.
      */
      template < typename T >
      inline void cholesky_solve(
        ublas::compressed_matrix<T> const & A
        , ublas::vector<T> & x
        , ublas::vector<T> const & b
        )
      {
        ublas::compressed_matrix<double> L(A);
        OpenTissue::math::big::incomplete_cholesky_decompose(L);
        x = b;
        OpenTissue::math::big::cholesky_solve(L, x, ublas::lower());
      }

      /**
      * Solve Linear System A x = b using Cholesky Factorization.
      *
      * @param A   The matrix (must be postive definite),
      * @param x   Upon return this argument contains the solution.
      * @param b   The right hand side vector.
      */
      template < typename T >
      inline void cholesky_solve(
        ublas::matrix<T> const & A
        , ublas::vector<T> & x
        , ublas::vector<T> const & b
        )
      {
        ublas::matrix<double> L(A);
        OpenTissue::math::big::cholesky_decompose(L);
        x = b;
        OpenTissue::math::big::cholesky_solve(L, x, ublas::lower());
      }

    } // namespace big
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_CHOLESKY_H
#endif
