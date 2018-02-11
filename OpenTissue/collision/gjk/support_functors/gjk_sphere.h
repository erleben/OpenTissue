#ifndef OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_SPHERE_H
#define OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_SPHERE_H
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
    * A Sphere.
    */
    template<typename math_types>
    class Sphere
    {
    protected:

      typedef typename math_types::real_type     T;
      typedef typename math_types::vector3_type  V;
      typedef typename math_types::value_traits  value_traits;

      T m_radius;  ///< Radius of sphere. Default value is one.

    public:

      /**
      * Get Radius.
      *
      * @return   A reference to the radius of the sphere.
      */
      T const & radius() const { return this->m_radius; }
      T       & radius()       { return this->m_radius; }

    public:

      Sphere()
        : m_radius( value_traits::one() )
      {}

    public:

      V operator()(V const & v) const 
      {
        using std::sqrt;

        assert( this->m_radius > value_traits::zero() || !"Sphere::operator(): Radius was non-positive");

        T const vv = dot(v,v);
        assert( is_number(vv) || !"Sphere::operator(): NaN encountered");

        if (vv > value_traits::zero() )
        {
          T const tmp =  this->m_radius / sqrt(vv);
          assert( is_number(tmp) || !"Sphere::operator(): NaN encountered");

          return v*tmp;
        }
        return V( this->m_radius, value_traits::zero(),  value_traits::zero());
      }

    };


  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_SPHERE_H
#endif
