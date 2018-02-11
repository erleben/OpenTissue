#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_AGGLOMERATE_VECTOR_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_AGGLOMERATE_VECTOR_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <stdexcept>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {
      /**
      * Transfer partitioned vector back to the global un-partitioned system.
      *
      * @param x_a          A sub-vector that should be transfered into the higher-dimensional vector x.
      * @param x_b          A sub-vector that should be transfered into the higher-dimensional vector x.
      * 
      * @param new2old       This vector holds an index permutation of the new indices (in the sub-vectors)
      *                      to the old indices (in the un-partitioned system). 
      *
      * @param x             Upon return this argument holds the agglomerated vector x = pi( x_a, x_b)
      */
      template<typename T>
      inline void agglomerate_vector(
        ublas::vector<T> const & x_a
        , ublas::vector<T> const & x_b
        , ublas::vector<size_t> const & new2old
        , ublas::vector<T> & x
        )
      {
        size_t n   = x.size();
        size_t n_a = x_a.size();
        size_t n_b = x_b.size();

        if ( (n_a + n_b) != n )
          throw std::invalid_argument( "Incompatible dimensions" );

        for(size_t i = 0; i< n_a;++i)
          x( new2old( i ) ) = x_a( i );
        for(size_t i = 0; i< n_b;++i)
          x( new2old( i + n_a ) ) = x_b( i );
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_AGGLOMERATE_VECTOR_H
#endif
