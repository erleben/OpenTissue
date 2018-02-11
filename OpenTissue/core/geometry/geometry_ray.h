#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_RAY_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_RAY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * Ray Class.
    * Ray is defined by all points
    *
    *   r = P + t*V
    *
    * where
    *
    *   t >= 0
    */
    template< typename math_types_ >
    class Ray
    {
    public:

      typedef          math_types_                   math_types;
      typedef typename math_types::real_type         real_type;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::value_traits      value_traits;

    protected:

      vector3_type m_p;  ///< Origin of ray.
      vector3_type m_r;  ///< Direction of ray (should be a unit vector).

    public:

      Ray()
        : m_p(value_traits::zero(),value_traits::zero(),value_traits::zero())
        , m_r(value_traits::one(),value_traits::zero(),value_traits::zero())
      {}

    public:

      vector3_type       & p()       { return m_p; }
      vector3_type const & p() const { return m_p; }
      vector3_type       & r()       { return m_r; }
      vector3_type const & r() const { return m_r; }

    public:

      void compute_point(real_type const & t)
      {
        assert(t>=value_traits::zero() || !"Ray::compute_point(): t<0, point not on ray" );
        return m_p + m_r*t;
      }

    };

  } // namespace geometry

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_RAY_H
#endif
