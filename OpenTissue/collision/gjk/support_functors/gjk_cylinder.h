#ifndef OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CYLINDER_H
#define OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CYLINDER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace gjk
  {
    /**
    * A Cylinder.
    */
    template<typename math_types>
    class Cylinder
    {
    protected:

      typedef typename math_types::real_type     T;
      typedef typename math_types::vector3_type  V;
      typedef typename math_types::value_traits  value_traits;

      T m_half_height;     ///< The half height of the cylinder. Default value is one.
      T m_radius;          ///< The radius of the cylinder. The default value is one.

    public:

      /**
      * Get Cylinder Half height.
      *
      * @return    A reference to the cylinder half height.
      */
      T const & half_height() const { return this->m_half_height; }
      T       & half_height()       { return this->m_half_height; }

      /**
      * Get Cylinder radius.
      *
      * @return    A reference to the cylinder radius.
      */
      T const & radius() const { return this->m_radius; }
      T       & radius()       { return this->m_radius; }

    public:

      Cylinder()
        : m_half_height( value_traits::one() )
        , m_radius( value_traits::one() )
      {}

    public:

      V operator()(V const & v) const
      {
        using std::sqrt;

        assert( this->m_half_height >= value_traits::zero() || !"Cylinder::operator(): Negative half height");
        assert( this->m_radius >= value_traits::zero()      || !"Cylinder::operator(): Negative radius");

        T const norm_sigma     = sqrt( v(0)*v(0) + v(1)*v(1) );
        assert( is_number( norm_sigma ) || !"Cylinder::operator(): NaN encountered");
        assert( norm_sigma >= value_traits::zero() || !"Cylinder::operator(): Norm can not be negative");

        T const h = v(2) > value_traits::zero() ? this->m_half_height : -this->m_half_height;
        assert( is_number( h ) || !"Cylinder::operator(): NaN encountered");

        // Test if search direction has any radial component
        if(norm_sigma > value_traits::zero() )
        {          
          V const s = V( 
              ((this->m_radius) * v(0)) /norm_sigma
            , ((this->m_radius) * v(1)) /norm_sigma
            , h 
            );

          assert( is_number( s(0) ) || !"Cylinder::operator(): NaN encountered");
          assert( is_number( s(1) ) || !"Cylinder::operator(): NaN encountered");
          assert( is_number( s(2) ) || !"Cylinder::operator(): NaN encountered");

          return s;
        }

        // Search direction is parallel with z-axis
        //
        // Or search direction is zero in which case we just some point as the support point.
        return V( value_traits::zero(), value_traits::zero(), h );
      }

    };

    
  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_CYLINDER_H
#endif
