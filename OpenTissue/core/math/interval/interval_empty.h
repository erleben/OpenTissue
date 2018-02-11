#ifndef OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_EMPTY_H
#define OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_EMPTY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/interval/interval_fwd.h>  //--- Needed for forward declaration of interval class

namespace OpenTissue
{
  namespace math
  {
    namespace interval
    {

      /**
      * Interval Empty Test.
      *
      * @param i     The interval.
      *
      * @return      If interval is empty then the return value is true otherwise it is false.
      */
      template<typename value_type>
      inline bool empty( Interval<value_type> const & i) {    return i.is_empty();  }

    } // namespace interval
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_INTERVAL_INTERVAL_EMPTY_H
#endif 
