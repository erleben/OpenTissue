#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_PD_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_PD_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_random.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_generate_PSD.h>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Generate Symmetric Positive Definite (PD) Matrix.
      * This function is a convenience function that is usefull
      * for quick-and-dirty initialization.
      *
      * @param n          The number of rows and columns in the resulting matrix.
      * @param A          Upon return this argument holds an n-by-n PD matrix.
      */
      template<typename matrix_type>
      inline void generate_PD(  size_t const & n,  matrix_type & A  )
      {
        typedef typename matrix_type::value_type                     value_type;
        typedef          OpenTissue::math::ValueTraits<value_type>   value_traits;

        generate_PSD(n, A, value_traits::zero() );
      }

      /**
      * Generate Symmetric Positive Definite (PD) Matrix.
      * This function is a convenience function that is usefull
      * for quick-and-dirty initialization.
      *
      * @param n          The number of rows and columns in the resulting matrix.
      * @param A          Upon return this argument holds an n-by-n PD matrix.
      */
      template<typename matrix_type>
      inline void fast_generate_PD(  size_t const & n,  matrix_type & A  )
      {
        typedef typename matrix_type::value_type                     value_type;
        typedef          OpenTissue::math::ValueTraits<value_type>   value_traits;

        Random<value_type> value(value_traits::zero(),value_traits::one());

        matrix_type R;
        R.resize(n,n,false);

        for(size_t i=0;i<n;++i)
        { 
          for(size_t j=0;j<n;++j)
            R(i,j) = value();
        }
        // Make A-matrix at least PSD
        ublas::noalias(A) = ublas::sparse_prod< matrix_type >( ublas::trans(R), R );
        // Make sure that A-matrix becomes PD
        for(size_t i=0;i<n;++i)
          A(i,i) += value();
      }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_GENERATE_PD_H
#endif
