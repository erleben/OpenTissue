#ifndef OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_VISCOUS_H
#define OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_VISCOUS_H
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
    class Viscous
    : public edm_types::force_type
    {
    public:
      typedef typename edm_types::force_type    base_type;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;
      typedef typename edm_types::Particle   particle_type;
      
    private:
      
      real_type  m_c;  ///< strength of the fluid force
      vector3_type  m_u;  ///< some constant stream velocity
      
      
    public:
      
      Viscous() 
      : base_type()
      , m_c(0)
      , m_u(0)
      {}
      
      virtual ~Viscous() {}
      
      void set(real_type const & c, vector3_type const & u)
      {
        m_c = c;
        m_u = u;
      }
      
    private:
      
      vector3_type apply(particle_type const & a) const
      {
        const vector3_type v(m_u-a.v);
        return vector3_type(m_c*(a.n*v)*a.n);
      }
      
    };
    
  }  // namespace edm
  
}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_VISCOUS_H
#endif
