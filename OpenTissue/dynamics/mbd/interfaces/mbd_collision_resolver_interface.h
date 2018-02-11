#ifndef OPENTISSUE_DYNAMICS_MBD_MBD_COLLISION_RESOLVER_INTERFACE_H
#define OPENTISSUE_DYNAMICS_MBD_MBD_COLLISION_RESOLVER_INTERFACE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/interfaces/mbd_stepper_interface.h>

namespace OpenTissue
{
  namespace mbd
  {

    template< typename mbd_types  >
    class CollisionResolverInterface : public StepperInterface<mbd_types>
    {
    public:

      typedef typename mbd_types::math_policy::real_type     real_type;
      typedef typename mbd_types::group_type                group_type;

      CollisionResolverInterface(){}
      virtual ~CollisionResolverInterface(){}

    public:

      void error_correction(group_type & group)
      {
        assert(false || !"CollisionResolverInterface::error_correction(): usually not defined for a collision resolver");
      }

      void run(group_type & group,real_type const & time_step)
      {
        assert(false || !"CollisionResolverInterface::run(): usually not defined for a collision resolver");
      }

    };

  } // namespace mbd
} // namespace OpenTissue
// OPENTISSUE_DYNAMICS_MBD_MBD_COLLISION_RESOLVER_INTERFACE_H
#endif
