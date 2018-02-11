#ifndef OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_BOX_H
#define OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_BOX_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {
    /**
    * A Box.
    */
    template<typename math_types>
    class Box
    {
    protected:

      typedef typename math_types::real_type     T;
      typedef typename math_types::vector3_type  V;
      typedef typename math_types::value_traits  value_traits;

      V m_half_extent;    ///< Half widht extents of the box.

    public:

      /**
      * Get Extents.
      *
      * @return    A reference to a vector holding the values of the box half width extents.
      */
      V const & half_extent() const { return this->m_half_extent; }
      V       & half_extent()       { return this->m_half_extent; }

    public:

      Box()
        : m_half_extent( value_traits::one(), value_traits::one(), value_traits::one() )
      {}

    public:

      V operator()(V const & v) const
      {
        assert( this->m_half_extent(0) >= value_traits::zero() || !"Box::operator(): Negative half extent encountered");
        assert( this->m_half_extent(1) >= value_traits::zero() || !"Box::operator(): Negative half extent encountered");
        assert( this->m_half_extent(2) >= value_traits::zero() || !"Box::operator(): Negative half extent encountered");

        T const vv = dot(v,v);
        assert( is_number(vv) || !"Box::operator(): NaN encountered");

        if (vv > value_traits::zero() )
        {

          T const c0 = v(0)>value_traits::zero() ? this->m_half_extent(0) : -this->m_half_extent(0);
          T const c1 = v(1)>value_traits::zero() ? this->m_half_extent(1) : -this->m_half_extent(1);
          T const c2 = v(2)>value_traits::zero() ? this->m_half_extent(2) : -this->m_half_extent(2);

          assert( is_number(c0) || !"Box::operator(): NaN encountered");
          assert( is_number(c1) || !"Box::operator(): NaN encountered");
          assert( is_number(c2) || !"Box::operator(): NaN encountered");

          return V(c0,c1,c2);
        }
        return (this->m_half_extent);
      }

    };

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_BOX_H
#endif
