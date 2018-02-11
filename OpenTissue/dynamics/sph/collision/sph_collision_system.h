#ifndef OPENTISSUE_DYNAMICS_SPH_COLLISION_SPH_COLLISION_SYSTEM_H
#define OPENTISSUE_DYNAMICS_SPH_COLLISION_SPH_COLLISION_SYSTEM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/collision/sph_collision_type.h>

#include <vector>

namespace OpenTissue
{
  namespace sph
  {

    template< typename CollisionDetectionPolicy >
    class CollisionSystem 
      : public CollisionDetectionPolicy
    {
    public:
      typedef CollisionDetectionPolicy  cd_policy;
      typedef typename cd_policy::real_type  real_type;
      typedef typename cd_policy::vector  vector;
      typedef typename cd_policy::point  point;
      typedef typename cd_policy::collision_type  collision_type;
      typedef std::vector<collision_type>  collision_container;

    };

  } // namespace sph

} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_COLLISION_SPH_COLLISION_SYSTEM_H
#endif
