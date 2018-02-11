#ifndef OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_GRAVITY_H
#define OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_GRAVITY_H
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
    class Gravity 
      : public types::force_type
    {
    public:

      typedef typename types::math_types            math_types;
      typedef typename math_types::real_type        real_type;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename types::system_type           system_type;

    protected:

      real_type  m_gravity; ///< Gravitational Acceleration.

    public:

      real_type       & gravity()       { return m_gravity; }
      real_type const & gravity() const { return m_gravity; }

    public:

      Gravity()
        : m_gravity(9.81)
      {}

      ~Gravity(){}

    public:

      void apply()
      {
        typedef typename system_type::particle_iterator   particle_iterator;
        using std::fabs;

        if(!(fabs(m_gravity) > 0))
          return;

        particle_iterator p   = this->owner()->particle_begin();
        particle_iterator end = this->owner()->particle_end();

        for(;p!=end;++p)
        {
          if( p->inv_mass() <= 0 )
            continue;
          p->force() += vector3_type(0,0, -m_gravity*p->mass());
        }
      }

    };

  } // namespace psys
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_FORCE_PSYS_GRAVITY_H
#endif
