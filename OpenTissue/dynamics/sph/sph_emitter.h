#ifndef OPENTISSUE_DYNAMICS_SPH_SPH_EMITTER_H
#define OPENTISSUE_DYNAMICS_SPH_SPH_EMITTER_H
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
  namespace sph
  {

    /**
    * SPH Particle Emitter Base Class.
    */
    template< typename Types >
    class Emitter
    {
    public:
      
      typedef typename Types::vector  vector;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_container  particle_container;
      typedef typename Types::particle_ptr_container  particle_ptr_container;

    public:
      /**
      * Default Constructor.
      */
      Emitter()
        : m_running(true) // auto start
        , m_rate(1)
        , m_currate(0)
        , m_batch(1)
        , m_particles(0)
      {}

      /**
      * Deconstructor.
      */
      virtual ~Emitter()
      {}

    public:

      /**
      * Active
      *
      * Return whether or not this emitter is still active, i.e. it still has unreleased particles.
      */
      bool active() const
      {
        return m_particles > 0;
      }

      /**
      * Running
      *
      * Return whether or not this emitter is still active, i.e. it still has unreleased particles.
      */
      bool running() const
      {
        return m_running;
      }

      /**
      * Rate
      */
      size_t & rate()
      {
        return m_rate;
      }

      /**
      * Rate (read only)
      */
      const size_t& rate() const
      {
        return m_rate;
      }

      /**
      * Batch
      */
      size_t& batch()
      {
        return m_batch;
      }

      /**
      * Batch (read only)
      */
      const size_t& batch() const
      {
        return m_batch;
      }

      virtual const vector& center() const = 0;

      /**
      * Initialize
      *
      * @param begin  the first particle iterator.
      * @param end  the last particle iterator.
      */
      bool initialize(typename particle_container::iterator begin, typename particle_container::iterator end)
      {
        m_pars.clear();
        m_particles = 0;
        for (typename particle_container::iterator par = begin; par != end; ++par) {
          par->fixed() = true;
          m_pars.push_back(&*par);
          ++m_particles;
        }
        return init();
      }

      /**
      * Start
      *
      * Starts the emitter.
      */
      void start()
      {
        m_running = true;
      }

      /**
      * Execute
      *
      * 
      */
      bool execute()
      {
        if (!m_running || !m_particles)
          return false;

        if (++m_currate < m_rate)
          return true;

        const size_t new_particles = m_particles < m_batch ? 0 : m_particles - m_batch;
        typename particle_ptr_container::iterator par = m_pars.begin()+(m_pars.size()-m_particles);
        typename particle_ptr_container::iterator end = new_particles ? par+m_batch : m_pars.end();
        for (; par != end; ++par)
          (*par)->fixed() = false;

        m_currate = 0;
        m_particles = new_particles;

        return true;
      }

      /**
      * Stop
      *
      * Stops the emitter (how odd).
      */
      void stop()
      {
        m_running = false;
      }

    private:
      /**
      * Initialize
      *
      */
      virtual bool init() = 0;

    protected:
      bool  m_running;
      size_t  m_rate;   /// release rate, execution frequency
      size_t  m_currate;   /// release rate, execution frequency
      size_t  m_batch;
      size_t  m_particles;
      particle_ptr_container  m_pars;

    }; // End class Emitter

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_SPH_EMITTER_H
#endif
