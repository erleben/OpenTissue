#ifndef OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_PIN_H
#define OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_PIN_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/psys_constraint.h>
#include <cassert>

namespace OpenTissue
{
  namespace psys
  {

    template<typename types>
    class Pin
      : public Constraint< types >
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::particle_type         particle_type;

    protected:

      particle_type  * m_particle;    ///< The particle that is subject to the constraint.
      vector3_type     m_r;           ///< The constrained position.

    public:

      vector3_type       & pin_position()       { return m_r; }
      vector3_type const & pin_position() const { return m_r; }

    public:

      Pin()
        : m_particle(0)
        , m_r(0,0,0)
      {}

      virtual ~Pin(){}

    public:

      /**
      * NOTE: Sets the pin position to the particle position!
      */
      void init(particle_type * particle)
      {
        assert(particle || !"Pin::init(): particle was null?");

        m_particle = particle;
        m_r = m_particle->position();
      }

      void satisfy()
      {
        if(!m_particle)
          return;
        m_particle->position() = m_r;
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_CONSTRAINTS_PSYS_PIN_H
#endif
