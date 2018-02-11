#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_IS_ORTHONORMAL_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_IS_ORTHONORMAL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <boost/cast.hpp>             // needed for boost::numeric_cast

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Orthonormal Testing.
      * This function is intended for debugging purposes it has
      * not been implemented with focus on performance or accuracy.
      * The function simply takes a brute force approach and tests
      * if all the columns of the specified matrix are orthonormal.
      *
      * @param A    The matrix that should be tested.
      *
      * @return     If the specified matrix is orthonormal then the
      *             return value is true otherwise it is false.
      */
      template<typename matrix_type>
      inline bool is_orthonormal(  matrix_type const & A  )
      {
        using namespace ublas;

        using std::fabs;

        typedef typename matrix_type::value_type                     value_type;
        typedef          OpenTissue::math::ValueTraits<value_type>   value_traits;
        typedef typename matrix_type::size_type                      size_type;

        size_type  const & m       = A.size1();
        size_type  const & n       = A.size2();

        assert( m>0         || !"is_orthonormal(): m was out of range");
        assert( n>0         || !"is_orthonormal(): n was out of range");
        assert( m==n        || !"is_orthonormal(): m and n was not equal");

        value_type const precision = ::boost::numeric_cast<value_type>(10e-6);

        for ( size_type i = 0; i < n; ++i )
        {
          value_type tmp = inner_prod( column(A,i), column(A,i) );
          if( fabs(tmp-value_traits::one()) > precision )
            return false;
        }
        for ( size_type i = 0; i < n; ++i )
          for ( size_type j = i+1; j < n; ++j )
          {
            value_type tmp = inner_prod( column(A,i), column(A,j) );
            if( fabs(tmp) >  precision )
              return false;
          }
          return true;
      }


    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_IS_ORTHONORMAL_H
#endif
