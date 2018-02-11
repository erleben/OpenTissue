#ifndef OPENTISSUE_DYNAMICS_PSYS_PSYS_H
#define OPENTISSUE_DYNAMICS_PSYS_PSYS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/collision/sdf/sdf.h>
#include <OpenTissue/collision/aabb_tree/aabb_tree.h>

#include <OpenTissue/dynamics/psys/psys_particle.h>
#include <OpenTissue/dynamics/psys/psys_system.h>
#include <OpenTissue/dynamics/psys/psys_constraint.h>
#include <OpenTissue/dynamics/psys/psys_contact_point.h>
#include <OpenTissue/dynamics/psys/psys_force.h>
#include <OpenTissue/dynamics/psys/psys_geometry_holder.h>
#include <OpenTissue/dynamics/psys/psys_connector_facade.h>
#include <OpenTissue/dynamics/psys/util/psys_direct_mesh_coupling.h>
#include <OpenTissue/dynamics/psys/forces/psys_gravity.h>
#include <OpenTissue/dynamics/psys/forces/psys_viscosity.h>
#include <OpenTissue/dynamics/psys/forces/psys_grid_force_field.h>
#include <OpenTissue/dynamics/psys/forces/psys_spring.h>
#include <OpenTissue/dynamics/psys/forces/psys_pressure_softbody.h>
#include <OpenTissue/dynamics/psys/forces/util/psys_compute_random_force_field.h>
#include <OpenTissue/dynamics/psys/forces/util/psys_compute_perlin_noise_force_field.h>
#include <OpenTissue/dynamics/psys/constraints/psys_pin.h>
#include <OpenTissue/dynamics/psys/constraints/psys_stick.h>
#include <OpenTissue/dynamics/psys/constraints/psys_box.h>
#include <OpenTissue/dynamics/psys/integrators/psys_verlet_integrator.h>
#include <OpenTissue/dynamics/psys/integrators/psys_euler_integrator.h>
#include <OpenTissue/dynamics/psys/mass_spring_system/psys_mass_spring_system.h>
#include <OpenTissue/dynamics/psys/mass_spring_system/psys_surface_mesh.h>
#include <OpenTissue/dynamics/psys/mass_spring_system/psys_cloth.h>
#include <OpenTissue/dynamics/psys/collision/collision_psys_plane.h>
#include <OpenTissue/dynamics/psys/collision/collision_psys_sphere.h>
#include <OpenTissue/dynamics/psys/collision/collision_psys_sdf.h>

namespace OpenTissue
{

  namespace psys
  {


    // TODO: It might be better to use a pimpl design principle for particle system integrators.
    //       currently there is one problem with changing this. The integrators are implemented
    //       using a template member method. Because they can be applied to different types of
    //       particle systems... How do we change this into a traditional abstract base class
    //       implementation?
    template<
        typename math_types_
      , typename integrator_policy = VerletIntegrator
    >
    class Types
    {
    public:

      typedef Types<math_types_,integrator_policy>  types;
      typedef math_types_                                 math_types;

      typedef VerletIntegrator              verlet_integrator;
      typedef EulerIntegrator               euler_integrator;

      typedef Particle<types>               particle_type;
      typedef System<types>                 system_type;

      typedef ContactPoint<types>           contact_point_type;
      typedef ConnectorFacade<  types >     connector_type;

      typedef Gravity<types>                gravity_type;
      typedef GridForceField<types>         grid_force_field_type;
      typedef PressureSoftBody<types>       pressure_soft_body_type;
      typedef Spring<types>                 spring_type;
      typedef Viscosity<types>              viscosity_type;
      typedef Force< types >                force_type;

      typedef GeometryHolder< types >       geometry_holder_type;

      typedef DirectMeshCoupling<types>     coupling_type;

      // TODO: grid_type is really bad naming it conflicts with grid_type of GridForceField, maybe it should be called phi_type?
      typedef OpenTissue::grid::Grid<float,math_types>             grid_type;

      typedef polymesh::PolyMesh<>              mesh_type;

      // TODO: Maybe ``system'' is redundant in these type names?
      typedef Cloth<types,integrator_policy>              cloth_system_type;
      typedef SurfaceMesh<types,integrator_policy>        surface_system_type;
      typedef MassSpringSystem<types,integrator_policy>   mass_spring_system_type;

      typedef Constraint<types> constraint_type;
      typedef Box<types>        box_constraint_type;
      typedef Stick<types>      stick_constraint_type;
      typedef Pin<types>        pin_constraint_type;

      // TODO: It is not strictly necessary for the typebinder to include the geometry types
      //       one just need to add set-methods and dispatch entries to the geometry
      //       holder class.
      typedef geometry::Sphere<math_types>           sphere_type;
      typedef geometry::Plane<math_types>            plane_type;
      typedef geometry::AABB<math_types>             aabb_type;
      typedef OpenTissue::sdf::Geometry<mesh_type,grid_type>       sdf_geometry_type;

      typedef OpenTissue::aabb_tree::Geometry<typename math_types::real_type,particle_type>  aabb_tree_type;
    };

  }//namespace psys
}//namespace OpenTissue

// OPENTISSUE_DYNAMICS_PSYS_PSYS_H
#endif
