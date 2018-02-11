#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_PSD_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_PSD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_generate_random.h>
#include <OpenTissue/core/math/big/big_gram_schmidt.h>
#include <OpenTissue/core/math/big/big_diag.h>
#include <OpenTissue/core/math/math_value_traits.h>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Generate Symmetric Positive Semi-Definite (PSD) Matrix.
      * This function is a convenience function that is usefull
      * for quick-and-dirty initialization.
      *
      * Notice, The eigenvalues of the resulting matrix is generated from
      * random values between zero and one.
      *
      * @param n          The number of rows and columns in the resulting matrix.
      * @param fraction   The (stocastic) fraction of zero-valued eigenvalues. If
      *                   this is set to zero then the matrix is a positive definite
      *                   matrix. If the fraction is set close to one then the number
      *                   of zero-eigenvalues grow very large.
      * @param A          Upon return this argument holds an n-by-n PSD matrix.
      */
      template<typename matrix_type>
      inline void generate_PSD(  size_t const & n,  matrix_type & A, typename matrix_type::value_type const & fraction  )
      {
        typedef typename matrix_type::value_type                     value_type;
        typedef          OpenTissue::math::ValueTraits<value_type>   value_traits;
        typedef          ublas::vector<value_type>                   vector_type;

        assert( fraction >= value_traits::zero() || !"generate_PSD(): invalid fraction specified");
        assert( fraction <= value_traits::one()  || !"generate_PSD(): invalid fraction specified");
        assert( n > 0                            || !"generate_PSD(): invalid problem size specified");

        Random<value_type> value(value_traits::zero(),value_traits::one());

        matrix_type Q;
        matrix_type D;
        matrix_type M;
        vector_type d;
        OpenTissue::math::big::generate_random( n, d );
        if(fraction>value_traits::zero())
        {
          for(size_t i = 0;i< n;++i)
          {
            if(value()<fraction)
              d(i) = value_traits::zero();
          }
        }
        OpenTissue::math::big::diag( d, D );
        OpenTissue::math::big::generate_random(n, n, Q);
        OpenTissue::math::big::gram_schmidt(Q);
        A.resize(n,n,false);
        M.resize(n,n,false);
        M = ublas::prod(D, ublas::trans(Q) );
        A = ublas::prod( Q, M );
      }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_PSD_H
#endif
