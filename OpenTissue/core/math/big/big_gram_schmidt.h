#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_GRAM_SCHMIDT_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_GRAM_SCHMIDT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#include <boost/numeric/ublas/vector_expression.hpp>
#include <boost/numeric/ublas/matrix_expression.hpp>

namespace ublas = boost::numeric::ublas;

#include <OpenTissue/core/math/math_value_traits.h>


namespace OpenTissue
{
  namespace math
  {
    namespace big
    {
      /**
      * Gram-Schmidt Orthonormalization.
      * This function orthonormalizes all the columns of the specified matrix.
      *
      * @param A  Upon invokation this argument holds the matrix that
      *           should be orthonormalized. Upon return the argument
      *           holds the orthonormalized matrix. Thus the orthonormalization
      *           is done in-place.
      */
      template<typename matrix_type>
      inline void gram_schmidt(  matrix_type & A  )
      {
        using namespace ublas;

        using std::fabs;

        typedef typename matrix_type::value_type                     value_type;
        typedef typename matrix_type::size_type                      size_type;
        typedef          OpenTissue::math::ValueTraits<value_type>   value_traits;

        size_type  const & m       = A.size1();
        size_type  const & n       = A.size2();

        assert( m>0         || !"gram_schmidt(): m was out of range");
        assert( n>0         || !"gram_schmidt(): n was out of range");
//        assert( m==n        || !"gram_schmidt(): A was not square");

        for (size_type k = 0; k < n; ++k ) 
        {
          value_type lgth = ublas::norm_2( column(A,k) );
          if( ! (fabs(lgth)> value_traits::zero()) )
            return;
          column(A,k) = column(A,k) / lgth;
          for (size_type  j = k+1; j < n; ++j )
          {
            value_type dot = ublas::inner_prod ( column(A,k), column(A,j) );
            column(A,j) -= dot*column(A,k);
          }
        }

      }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_GRAM_SCHMIDT_H
#endif
