#ifndef OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CAPSULE_H
#define OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CAPSULE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/gjk/support_functors/gjk_sphere.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {
    /**
    * A Capsule.
    */
    template<typename math_types>
    class Capsule
    {
    protected:

      typedef typename math_types::real_type     T;
      typedef typename math_types::vector3_type  V;
      typedef typename math_types::value_traits  value_traits;

      T m_half_height;     ///< The half height of the Capsule. Default value is one.
      T m_radius;          ///< The radius of the Capsule. The default value is one.

    public:

      /**
       * Capsule half-height.
       */
      T const & half_height() const { return this->m_half_height; }
      T       & half_height()       { return this->m_half_height; }

      /**
       * Capsule raius.
       */
      T const & radius() const { return this->m_radius; }
      T       & radius()       { return this->m_radius; }

    public:

      Capsule()
        : m_half_height( value_traits::one() )
        , m_radius( value_traits::one() )
      {}

    public:

      V operator()(V const & v) const
      {
        assert( this->m_half_height >= value_traits::zero() || !"Capsule::operator(): Negative half height");
        assert( this->m_radius >= value_traits::zero()      || !"Capsule::operator(): Negative radius");

        Sphere<math_types> S;
        S.radius() = this->m_radius;

        // Get the support point of the sphere
        V w = S(v);

        // Cut the sphere into two halves and displace them along the z-axis.
        if( v(2) > value_traits::zero() )
          w(2) += this->m_half_height;
        else if( v(2) < value_traits::zero() )
          w(2) -= this->m_half_height;

        assert( is_number( w(0) ) || !"Capsule::operator(): NaN encountered");
        assert( is_number( w(1) ) || !"Capsule::operator(): NaN encountered");
        assert( is_number( w(2) ) || !"Capsule::operator(): NaN encountered");

        return w;
      }

    };

  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CAPSULE_H
#endif
