#ifndef OPENTISSUE_DYNAMICS_SPH_EMITTERS_SPH_CIRCLE_H
#define OPENTISSUE_DYNAMICS_SPH_EMITTERS_SPH_CIRCLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/sph_emitter.h>
#include <OpenTissue/core/math/math_constants.h>

namespace OpenTissue
{
  namespace sph
  {

    // TODO: Missing some stuff about circle orientation.
    //       The velocity vector could be used, and the circle is just on z = 0 if v = 0.

    /**
    * SPH Particle Circle Emitter.
    */
    template< typename Types >
    class CircleEmitter : public Emitter<Types>
    {
    public:
      typedef Emitter<Types>  base_type;
      typedef typename Types::real_type  real_type;
      typedef typename Types::vector  vector;
      typedef typename Types::particle  particle;
      typedef typename Types::particle_ptr_container  particle_ptr_container;

    public:

      CircleEmitter(vector const & center, real_type const & radius, vector const & velocity = 0) 
        : base_type()
        , m_c(center)
        , m_r(radius)
        , m_v(velocity)
      {}

      ~CircleEmitter()
      {}

    public:
      vector const & center() const {  return m_c; }

    private:

      /**
      * Initialize
      */
      bool init()
      {
        const real_type Dt = 2.*math::detail::pi<real_type>()/base_type::m_batch;
        real_type t = 0;
        typename particle_ptr_container::iterator end = base_type::m_pars.end();
        for (typename particle_ptr_container::iterator par = base_type::m_pars.begin(); par != end; ++par) {
          particle* p = *par;
          p->velocity() = m_v;
          vector disp; 
          random(disp,-0.001, 0.001);
          p->position() = m_c+disp+m_r*vector(cos(t), sin(t), 0);
          t += Dt;
        }
        return true;
      }

    protected:

      vector     m_c;   /// release point.
      real_type  m_r;   /// circle radius.
      vector     m_v;   /// release velocity.

    }; // End class PointEmitter

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_EMITTERS_SPH_CIRCLE_H
#endif
