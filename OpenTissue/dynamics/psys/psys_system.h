#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_SYSTEM_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <vector>

namespace OpenTissue
{
  namespace psys
  {


    /**
     * This is the ``core'' data structure for particles. It provides iterator
     * capabilities. Particle lookup based on indices and time-management of
     * the entire particle system.
     *
     * Also there is simple support functionality for deriving AABB information
     * of the particle system (see min and max methods).
     */
    template<typename types >
    class System
    {
    public:

      typedef typename types::math_types           math_types;
      typedef typename math_types::real_type       real_type;
      typedef typename math_types::vector3_type    vector3_type;
      typedef typename types::particle_type        particle_type;

      typedef          std::vector<particle_type>           particle_container;
      typedef typename particle_container::iterator         particle_iterator;
      typedef typename particle_container::const_iterator   const_particle_iterator;

    protected:

      real_type            m_time;         ///< Current time.
      particle_container   m_particles;    ///< A vector of all particles in the cluster.

    public:

      real_type       & time()       { return m_time; }
      real_type const & time() const { return m_time; }

    public:

      System()
        : m_time(0.0)
      {}

      ~System(){  clear(); };

    public:

      vector3_type min_coord() 
      {
        using std::min;

        // TODO: Is it possible to add some lazy evalutation to this? Caching
        // previous computed value and only re-compute it if particles have
        // changed?
        vector3_type min_coord = vector3_type( math::detail::highest<real_type>(),math::detail::highest<real_type>(),math::detail::highest<real_type>() );
        particle_iterator p = particle_begin();
        particle_iterator end = particle_end();
        for(;p!=end;++p)
          min_coord = min( min_coord, p->position() );
      }

      vector3_type max_coord() 
      {
        using std::max;

        // TODO: Is it possible to add some lazy evalutation to this? Caching
        // previous computed value and only re-compute it if particles have
        // changed?
        vector3_type max_coord = vector3_type( math::detail::lowest<real_type>(),math::detail::lowest<real_type>(),math::detail::lowest<real_type>() );
        particle_iterator p = particle_begin();
        particle_iterator end = particle_end();
        for(;p!=end;++p)
          max_coord = max( max_coord, p->position() );
      }

    public:

      void clear()
      {
        m_particles.clear();
      }

      particle_iterator create_particle(particle_type const & p)
      {
        m_particles.push_back(p);
        particle_iterator particle = m_particles.end()-1;
        particle->connect ( *this );
        return particle;
      }

      void erase(particle_iterator p) 
      { 
        p->disconnect();
        m_particles.erase(p); 
      }

      particle_iterator       & operator()(unsigned int idx)       { return m_particles.begin() + idx; }
      const_particle_iterator & operator()(unsigned int idx) const { return m_particles.begin() + idx; }

      particle_iterator       particle_begin()       { return m_particles.begin(); }
      particle_iterator       particle_end()         { return m_particles.end(); }
      const_particle_iterator particle_begin() const { return m_particles.begin(); }
      const_particle_iterator particle_end()   const { return m_particles.end();   }

      std::size_t particles_size() const { return m_particles.size(); }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PSYS_SYSTEM_H
#endif
