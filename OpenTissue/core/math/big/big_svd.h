#ifndef OPENTISSUE_CORE_MATH_BIG_SVD_H
#define OPENTISSUE_CORE_MATH_BIG_SVD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#ifdef USE_ATLAS
#  include <OpenTissue/core/math/big/big_svd_impl_atlas.h>  
#else
#  include <OpenTissue/core/math/big/big_svd_impl1.h>  
#endif 
#include <OpenTissue/core/math/big/big_types.h>  
#include <OpenTissue/core/math/math_value_traits.h>  
#include <boost/cast.hpp>             // needed for boost::numeric_cast
#include <cmath>                      // needed for std::fabs
#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {
      
      /**
       * Compute Singular Value Decomposition of a matrix.
       *
       *     A = U S VT
       *
       * @param A          The matrix.
       * @param U          Upon return this matrix holds orthogonal columns.
       * @param s          A vector containing the singular values.
       * @param V         Upon return this matrix is an orthogonal matrix.
       *
       */
      template<typename ME>
      inline void svd( 
                      ublas::matrix_expression<ME> const & A
                      , ublas::matrix<typename ME::value_type> & U
                      , ublas::vector<typename ME::value_type> & s
                      , ublas::matrix<typename ME::value_type> & V
                      )
      {
#ifdef USE_ATLAS
        ublas::matrix<typename ME::value_type, ublas::column_major>  VT;
        ublas::matrix<typename ME::value_type, ublas::column_major>  UU;
        
        detail::svd_impl_atlas( A, UU, s, VT);
        
        V.resize( VT.size1(), VT.size2() ); 		
		    V = ublas::trans( VT );
        U = UU;
#else
        detail::svd_impl1(A, U, s, V);
#endif
      }
      
      
      /**
       * Singular Value Decomposition Solver.
       * First the function computes the singular value
       * decomposition, A = U D VT, then it solves for the
       * x-solution using this decomposition:
       * 
       *  x = V inv(D) (UT (b))
       *  
       *
       * @param A   The matrix.
       * @param x   Upon return this argument hold the solution to the linear system A x = b
       * @param b   The right hand side vector.
       *
       */
      template<typename matrix_type, typename vector_type>
      inline void svd( matrix_type const & A, vector_type & x, vector_type const & b )
      {
        using std::fabs;
        
        typedef typename matrix_type::size_type                size_type;
        typedef typename vector_type::value_type               value_type;
        typedef ublas::matrix<value_type>                      row_major_matrix_type;
        typedef OpenTissue::math::ValueTraits<value_type>      value_traits;
        
        if(A.size1() <= 0 || A.size2() <= 0)
          throw std::invalid_argument("svd(): A was empty");
        
        if(b.size() != A.size1())
          throw std::invalid_argument("svd(): The size of b must be the same as the number of rows in A");
        
        if(x.size() != A.size2())
          throw std::invalid_argument("svd(): The size of x must be the same as the number of columns in A");
        
        
        static value_type const tiny = ::boost::numeric_cast<value_type>( 10e-4);
        
        //size_type m = A.size1();
        size_type n = A.size2();
        
        //--- SVD decomposition  : A = U S VT : Dimensions: mxn =  mxm  mxn   nxn
        row_major_matrix_type U;
        vector_type s;
        row_major_matrix_type V;
        
        svd(A,U,s,V);
        
        //--- x = V inv(diag(s)) (UT (b))
        for ( size_type i = 0; i < n; ++i )
        {
          if( fabs( s( i ) ) < tiny )
            s( i ) = value_traits::zero();
          else
            s( i ) = value_traits::one() / s( i );
        }
        vector_type y = ublas::prod( ublas::trans( U ), b );
        x = ublas::prod( V, ublas::element_prod( s, y) );
      }
      
      
      /**
       * Matrix Inversion by Singular Value Decomposition.
       *
       * @param A     The matrix to be inverted.
       * @param invA  Upon return this argument holds the inverted matrix.
       *
       */
      template<class matrix_type>
      inline void svd_invert(matrix_type const & A, matrix_type& invA) 
      {
        using std::fabs;
        
        typedef typename matrix_type::size_type                size_type;
        typedef typename matrix_type::value_type               value_type;
        typedef ublas::vector<value_type>                      vector_type;
        typedef OpenTissue::math::ValueTraits<value_type>      value_traits;
        typedef ublas::matrix<value_type>                      row_major_matrix_type;
        
        if(A.size1() <= 0 || A.size2() <= 0)
          throw std::invalid_argument("svd(): A was empty");
        
        static value_type const tiny = ::boost::numeric_cast<value_type>( 10e-4);
        
        size_type m = A.size1();
        size_type n = A.size2();
        
        invA.resize(n,m,false);
        
        //--- SVD decomposition  : A = U S VT : Dimensions: mxn =  mxm  mxn   nxn
        row_major_matrix_type U;
        vector_type s;
        row_major_matrix_type V;
        svd(A,U,s,V);
        
        //--- x = V inv(diag(s)) (UT (b))
 
        // This is brain-death way of creating a diagonal matrix from a vector
        
        row_major_matrix_type D;
        D.resize(n,n);
        D.clear();
        for ( size_type i = 0; i < n; ++i )
        {
          if( fabs( s( i ) ) < tiny )
            D( i, i ) = value_traits::zero();
          else
            D( i, i ) = value_traits::one() / s( i );
        }
        
        row_major_matrix_type tmp = ublas::prod( D, ublas::trans(U) );
        invA.assign(   ublas::prod( V, tmp  )    );
      }
      
      
    } // namespace big
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_SVD_H
#endif
