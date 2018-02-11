#ifndef OPENTISSUE_DYNAMICS_MULIBODY_CD_MBD_SETUP_DEFAULT_GEOMETRY_DISPATCHER_H
#define OPENTISSUE_DYNAMICS_MULIBODY_CD_MBD_SETUP_DEFAULT_GEOMETRY_DISPATCHER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_box_box_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_sdf_sphere_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_box_plane_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_sphere_box_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_sphere_plane_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_sdf_plane_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_sphere_sphere_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_sdf_sdf_handler.h>
#include <OpenTissue/dynamics/mbd/collision_detection/collision_handlers/mbd_inverted_box_sphere_handler.h>

namespace OpenTissue
{
  namespace mbd
  {

    /**
    * Setup Default Geometry Dispatcher.
    * This function will initialize the collision matrix in the geometry dispatcher.
    *
    * @param simulator     A reference to the simulator where the dispatcher belongs to.
    */
    template<typename simulator_type>
    inline void setup_default_geometry_dispatcher( simulator_type & simulator)
    {
      typedef typename simulator_type::types                  mbd_types;

      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::BoxBoxHandler<mbd_types>::test   );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SDFSDFHandler<mbd_types>::test    );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SDFSphereHandler<mbd_types>::test  );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SDFSphereHandler<mbd_types>::mirrowed_test  );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SphereSphereHandler<mbd_types>::test  );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SDFPlaneHandler<mbd_types>::test );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SDFPlaneHandler<mbd_types>::mirrowed_test );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SpherePlaneHandler<mbd_types>::test );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SpherePlaneHandler<mbd_types>::mirrowed_test );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SphereBoxHandler<mbd_types>::test );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::SphereBoxHandler<mbd_types>::mirrowed_test );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::BoxPlaneHandler<mbd_types>::test );
      simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::BoxPlaneHandler<mbd_types>::mirrowed_test );
      //simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::InvertedBoxSphereHandler<mbd_types>::test );
      //simulator.get_collision_detection()->get_narrow_phase()->bind( &OpenTissue::mbd::collision_detection::InvertedBoxSphereHandler<mbd_types>::mirrowed_test );
    }

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MULIBODY_CD_MBD_SETUP_DEFAULT_GEOMETRY_DISPATCHER_H
#endif
