#ifndef OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_GRAVITY_H
#define OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_GRAVITY_H
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

  namespace edm
  {

    template<typename edm_types>
    class Gravity
      : public edm_types::force_type
    {
    public:

      typedef typename edm_types::force_type    base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::vector3_type  vector3_type;
      typedef typename edm_types::Particle      particle_type;

    private:
      
      vector3_type  m_gravity;  ///< the gravitational acceleration
      
    public:

      Gravity() 
        : base_type()
        , m_gravity(value_traits::zero())
      {}

      virtual ~Gravity() {}

      void set(vector3_type const & g)
      {
        m_gravity = g;
      }

    private:

      vector3_type apply(particle_type const & a) const
      {
        return vector3_type(m_gravity*a.m);
      }

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_GRAVITY_H
#endif
