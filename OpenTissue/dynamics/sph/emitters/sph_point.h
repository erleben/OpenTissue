#ifndef OPENTISSUE_DYNAMICS_SPH_EMITTERS_SPH_POINT_H
#define OPENTISSUE_DYNAMICS_SPH_EMITTERS_SPH_POINT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_emitter.h>

namespace OpenTissue
{
  namespace sph
  {

    /**
    * SPH Particle Point Emitter (Default).
    */
    template< typename Types >
    class PointEmitter : public Emitter<Types>
    {
    public:
      typedef Emitter<Types>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_ptr_container  particle_ptr_container;

    public:

      PointEmitter( vector const & point, vector const & velocity = 0)
        : base_type()
        , m_p(point)
        , m_v(velocity)
      {}

      ~PointEmitter()
      {}

    public:

      const vector& center() const  {  return m_p;  }

    private:

      /**
      * Initialize
      */
      bool init()
      {
        typename particle_ptr_container::iterator end = base_type::m_pars.end();
        for (typename particle_ptr_container::iterator par = base_type::m_pars.begin(); par != end; ++par) {
          particle* p = *par;
          p->velocity() = m_v;
          //        p->position() = m_p;
          vector disp;
          random(disp,-0.001, 0.001);
          p->position() = m_p+disp;
        }
        return true;
      }

    protected:

      vector  m_p;   /// release point
      vector  m_v;   /// release velocity

    }; // End class PointEmitter

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_EMITTERS_SPH_POINT_H
#endif
