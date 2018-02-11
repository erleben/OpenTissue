#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_INDEX_REORDERING_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_INDEX_REORDERING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_constants.h>
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      /**
      * Compute Reordering of Constraints
      *
      * @param bitmask       A bitmask vector indicating set membership of the variables. The
      *                      encoding is as follows:
      *
      *                      The i'th value is equal to ``in active'' if bitmask(i) = 4. In
      *                      terms of complementarity formulations this is true if and only
      *                      if  y(i) \leq (x(i) - lo(i)) &&  y(i) \geq (x(i) - hi(i))
      *
      *                      The i'th value is equal to ``in upper'' if bitmask(i) = 2. In
      *                      terms of complementarity formulations this is true if and only
      *                      if  y(i) < (x(i) - hi(i))
      *
      *                      The i'th value is equal to ``in lower'' if bitmask(i) = 1. In
      *                      terms of complementarity formulations this is true if and only
      *                      if  y(i) > (x(i) - lo(i))    
      *
      * @param old2new       Upon return this vector holds an index permutation of old indices
      *                      to new indices. The permutation is such that the first sub-block of
      *                      the new vector corresponds to active constraints. The second corresponds
      *                      to lower constraints and the last sub-block corresponds to the upper
      *                      constraints set.
      * @oaram new2old       Upon return this vector holds the reversible permuation of old2new.
      *
      */
      inline void  compute_index_reordering( 
        ublas::vector<size_t> const & bitmask
        , ublas::vector<size_t>    & old2new
        , ublas::vector<size_t>    & new2old
        )
      {
        size_t n = bitmask.size();
        old2new.resize(n);
        new2old.resize(n);

        size_t r = 0;

        for (size_t i = 0; i < n; ++ i)
        {
          if(bitmask(i) == IN_ACTIVE)
          {
            old2new( i ) = r;
            new2old( r ) = i;
            ++r;
          }
        }

        for (size_t i = 0; i < n; ++ i)
        {
          if(bitmask(i) == IN_LOWER)
          {
            old2new( i ) = r;
            new2old( r ) = i;
            ++r;
          }
        }

        for (size_t i = 0; i < n; ++ i)
        {
          if(bitmask(i) == IN_UPPER)
          {
            old2new( i ) = r;
            new2old( r ) = i;
            ++r;
          }
        }

        assert( r==n || !"compute_index_reordering(): something went wrong");
      }


    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_INDEX_REORDERING_H
#endif
