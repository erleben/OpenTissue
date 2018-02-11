#ifndef OPENTISSUE_CORE_MATH_BIG_BIG_DIAG_H
#define OPENTISSUE_CORE_MATH_BIG_BIG_DIAG_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>

namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Generate Diagonal Matrix.
      * This function is a convenience function that is usefull
      * for quick-and-dirty initialization.
      *
      * @param v          A vector which holds the diagonal values of the resulting matrix.
      * @param A          A square diagonal matrix holding the values of the vector along the diagonal.
      */
      template<typename value_type, typename matrix_type>
      inline void diag( ublas::vector<value_type> const & v, matrix_type & D  )
      {
        size_t const n = v.size();
        assert( n>0         || !"diag(): n was out of range");
        D.resize(n,n,false);
        D.clear();
        for(size_t i=0;i<n;++i)
          D(i,i) = v(i);
      }

    } // end of namespace big
  } // end of namespace math
} // end of namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_BIG_DIAG_H
#endif
