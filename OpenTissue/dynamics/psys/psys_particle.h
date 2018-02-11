#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_PARTICLE_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_PARTICLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/psys/psys_connector_facade.h>

namespace OpenTissue
{

  namespace psys
  {


    template<typename types>
    class Particle 
      : public ConnectorFacade< types >
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::system_type           system_type;

    protected:

      bool            m_coupled;      ///< Boolean flag indicating whether the particle position is coupled. Default is false.

      vector3_type  * m_ptr_r;

      vector3_type    m_r;            ///< A binding to the current position.
      vector3_type    m_old_r;        ///< The old position (ie. in last iteration of the particle system the particle is in).
      vector3_type    m_v;            ///< The current velocity.
      vector3_type    m_f;            ///< Total Force (Accumulator).
      vector3_type    m_a;            ///< Total acceleration. (F = m a -> a = F/m).
      real_type       m_inv_mass;     ///< Inverse mass (a value of 0 means a fixed particle)
      real_type       m_mass;         ///< The mass (a value of infinity means a fixed particle).


    public:

      Particle()
        : m_coupled (false)
        , m_ptr_r(0)
        , m_r(0,0,0)
        , m_old_r(0,0,0)
        , m_v(0,0,0)
        , m_f(0,0,0)
        , m_a(0,0,0)
        , m_inv_mass(1.0)
        , m_mass(1.0)

      {
      }

      void bind(vector3_type const & r) 
      { 
        m_ptr_r = const_cast<vector3_type*>(&r); 
        m_old_r = r;
        m_coupled = true;   
      }

      void release() 
      {
        if(m_coupled)
        {
          m_r = *m_ptr_r;
        }
        m_coupled = false;
      }

    public:

      vector3_type       & position()           { return m_coupled?*m_ptr_r:m_r; }
      vector3_type const & position() const     { return m_coupled?*m_ptr_r:m_r; }
      vector3_type       & old_position()       { return m_old_r;                }
      vector3_type const & old_position() const { return m_old_r;                }
      vector3_type       & velocity()           { return m_v;                    }
      vector3_type const & velocity() const     { return m_v;                    }
      vector3_type       & force()              { return m_f;                    }
      vector3_type const & force() const        { return m_f;                    }
      vector3_type       & acceleration()       { return m_a;                    }
      vector3_type const & acceleration() const { return m_a;                    }
      real_type          & inv_mass()           { return m_inv_mass;             }
      real_type const    & inv_mass() const     { return m_inv_mass;             }
      real_type          & mass()               { return m_mass;                 }
      real_type const    & mass() const         { return m_mass;                 }

    };

  } // namespace psys

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PARTICLE_H
#endif
