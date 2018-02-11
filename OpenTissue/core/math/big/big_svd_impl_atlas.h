#ifndef OPENTISSUE_CORE_MATH_BIG_SVD_IMPL_ATLAS_H
#define OPENTISSUE_CORE_MATH_BIG_SVD_IMPL_ATLAS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2009 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/traits/ublas_vector.hpp>
#include <boost/numeric/bindings/traits/ublas_symmetric.hpp>
#include <boost/numeric/bindings/lapack/gesvd.hpp>

namespace lapack = boost::numeric::bindings::lapack;

#include <OpenTissue/core/math/big/big_types.h>  

#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {
      namespace detail
      {
        
        /**
         * Compute Singular Value Decomposition of a matrix.
         *
         *     A = U S VT
         *
         * @param A          The matrix.
         * @param U          Upon return this matrix holds orthogonal columns.
         * @param s          A vector containing the singular values.
         * @param VT         Upon return this matrix is an orthogonal matrix.
         *
         */
        template<typename ME>
        inline void svd_impl_atlas( 
                                   ublas::matrix_expression<ME> const & A
                                   , ublas::matrix<typename ME::value_type, ublas::column_major> & U
                                   , ublas::vector<typename ME::value_type> & s
                                   , ublas::matrix<typename ME::value_type, ublas::column_major> & VT
                                   )
        {          
          typedef typename ME::size_type                         size_type;
          typedef typename ME::value_type                        value_type;
          typedef ublas::matrix<value_type, ublas::column_major> column_major_matrix_type;
          
          //--- SVD decomposition  : A = U S VT : Dimensions: mxn =  mxn  nxn   nxn
          size_type m = A().size1();
          size_type n = A().size2();
          
          if(m<1)
            throw std::invalid_argument("svd(): A did not have any rows?");
          if(n<1)
            throw std::invalid_argument("svd(): A did not have any columns?");
          
          column_major_matrix_type Acpy( m, n );
          Acpy = A();
          
          U.resize(m,n,false);
          s.resize(n,false);
          VT.resize(n,n,false);
          
          lapack::gesvd( Acpy, s, U, VT );
        }
        
        
      } // namespace detail
    } // namespace big
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_SVD_IMPL_ATLAS_H
#endif
