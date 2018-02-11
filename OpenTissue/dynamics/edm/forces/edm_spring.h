#ifndef OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_SPRING_H
#define OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_SPRING_H
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
    class Spring
      : public edm_types::force_type
    {
    public:

      typedef typename edm_types::force_type    base_type;
      typedef typename edm_types::value_traits  value_traits;
      typedef typename edm_types::real_type     real_type;
      typedef typename edm_types::vector3_type  vector3_type;
      typedef typename edm_types::Particle      particle_type;

    private:
      
      real_type  m_k; ///< the spring constant (Hooke's law)
      vector3_type  m_r0;  ///< the fixed connection point (in world coordinates)
      
    public:

      Spring() 
        : base_type()
        , m_k(value_traits::zero())
        , m_r0(value_traits::zero())
      {}

      virtual ~Spring() {}

      void set(real_type const & k, vector3_type const & r0)
      {
        m_k = k;
        m_r0 = r0;
      }

    private:

      vector3_type apply(particle_type const & a) const
      {
        return vector3_type(m_k*(m_r0-a.r));
      }

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_FORCES_EDM_SPRING_H
#endif
