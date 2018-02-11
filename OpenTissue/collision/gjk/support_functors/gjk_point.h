#ifndef OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_POINT_H
#define OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_POINT_H
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
    * A Point.
    */
    template<typename math_types>
    class Point
    {
    protected:

      typedef typename math_types::real_type     T;
      typedef typename math_types::vector3_type  V;
      typedef typename math_types::value_traits  value_traits;

      V m_point;  ///< The point.

    public:

      /**
      * Get Radius.
      *
      * @return   A reference to the radius of the sphere.
      */
      V const & point() const { return this->m_point; }
      V       & point()       { return this->m_point; }

    public:

      Point()
        : m_point( value_traits::zero(), value_traits::zero(), value_traits::zero() )
      {}

    public:

      V operator()(V const & v) const 
      {
        assert( is_number( this->m_point(0) ) || !"Point::operator(): NaN encountered");
        assert( is_number( this->m_point(1) ) || !"Point::operator(): NaN encountered");
        assert( is_number( this->m_point(2) ) || !"Point::operator(): NaN encountered");

        return this->m_point;
      }

    };


  } // namespace gjk

} // namespace OpenTissue

// OPENTISSUE_COLLISION_GJK_GJK_SUPPORT_FUNCTORS_POINT_H
#endif
