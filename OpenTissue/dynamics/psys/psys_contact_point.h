#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_CONTACT_POINT_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_CONTACT_POINT_H
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
  namespace psys
  {

    template<typename types>
    class ContactPoint
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;

    public:

      vector3_type m_p;               ///< Point of contact.
      vector3_type m_n;               ///< Direction of minimum translation (contact normal).
      real_type m_distance;           ///< Minimum Translation Distance (separation/penetration distance).

      particle_type * m_A0;
      particle_type * m_A1;
      particle_type * m_A2;

      particle_type * m_B0;
      particle_type * m_B1;
      particle_type * m_B2;

      real_type m_a0;
      real_type m_a1;
      real_type m_a2;

      real_type m_b0;
      real_type m_b1;
      real_type m_b2;

    public:

      ContactPoint()
        : m_p(0,0,0)
        , m_n(0,0,0)
        , m_distance()
        , m_A0(0)
        , m_A1(0)
        , m_A2(0)
        , m_B0(0)
        , m_B1(0)
        , m_B2(0)
        , m_a0()
        , m_a1()
        , m_a2()
        , m_b0()
        , m_b1()
        , m_b2()
      { }

    }; 

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PSYS_CONTACT_POINT_H
#endif
