#ifndef OPENTISSUE_DYNAMICS_EDM_EDM_FORCE_H
#define OPENTISSUE_DYNAMICS_EDM_EDM_FORCE_H
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
    class Force
    {
    public:

      typedef typename edm_types::vector3_type  vector3_type;
      typedef typename edm_types::Particle      particle_type;

    public:

      virtual ~Force() {}
    
    public:

      virtual vector3_type apply(particle_type const & a) const = 0;

    };

  }  // namespace edm

}  // namespace OpenTissue

// OPENTISSUE_DYNAMICS_EDM_EDM_FORCE_H
#endif
