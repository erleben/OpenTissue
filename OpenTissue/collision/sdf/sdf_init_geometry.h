#ifndef OPENTISSUE_COLLISION_SDF_SDF_INIT_GEOMETRY_H
#define OPENTISSUE_COLLISION_SDF_SDF_INIT_GEOMETRY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/sdf/sdf_compute_point_sampling.h>
#include <OpenTissue/collision/sdf/sdf_top_down_policy.h>

namespace OpenTissue
{

  namespace sdf
  {


    /**
    * Initialize Distance Map Geometry.
    *
    * The signed distance field and the mesh must be given wrt. the same coordinate frame system.
    *
    * It is implicitly assumed that the coordinate frame is the body frame, and it is also assumed that
    * the mesh and the signed distance field are in perfect alignment with each other.
    *
    * @param mesh              The surface mesh (corresponds to the zero-level set of phi).
    * @param phi               The signed distance field of the specified surface mesh.
    * @param edge_resolution   Threshold value, indicating the sampling
    *                          resolution along edges. If zero it will be
    *                          computed on the fly, to match the resolution
    *                          of the signed distance map.
    *
    * @param face_sampling     Boolean flag indicating wheter face sampling is on or off.
    * @param geometry          Upon return this argument holds the signed distance field geometry.
    *
    */
    template<typename mesh_type,typename grid_type,typename sdf_geometry_type>
    void init_geometry(
      mesh_type /*const*/ & mesh
      , grid_type & phi
      , double edge_resolution
      , bool face_sampling
      , sdf_geometry_type & geometry
      )
    {
      typedef typename sdf_geometry_type::bvh_type                        bvh_type;
      typedef typename sdf_geometry_type::vector3_type                    vector3_type;
      typedef typename vector3_type::value_type                           real_type;
      typedef          TopDownPolicy<bvh_type>                        top_down_policy;
      typedef          bvh::TopDownConstructor<bvh_type, top_down_policy >  constructor_type;

      geometry.m_mesh = mesh;
      geometry.m_phi = phi;

      geometry.m_sampling.clear();
      compute_point_sampling(mesh,phi, edge_resolution, face_sampling, geometry.m_sampling);

      geometry.m_min_coord(0) = math::detail::highest<real_type>();
      geometry.m_min_coord(1) = math::detail::highest<real_type>();
      geometry.m_min_coord(2) = math::detail::highest<real_type>();
      geometry.m_max_coord(0) = math::detail::lowest<real_type>();
      geometry.m_max_coord(1) = math::detail::lowest<real_type>();
      geometry.m_max_coord(2) = math::detail::lowest<real_type>();
      geometry.m_max_radius = 0;
      for(typename sdf_geometry_type::point_iterator p = geometry.m_sampling.begin();p!=geometry.m_sampling.end();++p)
      {     
        geometry.m_min_coord  = min (  geometry.m_min_coord, (*p) );
        geometry.m_max_coord  = max (  geometry.m_max_coord, (*p) );
        real_type distance = std::sqrt( (*p)*(*p)  );
        if(distance>geometry.m_max_radius)
          geometry.m_max_radius = distance;
      }

      geometry.m_bvh.clear();
      constructor_type constructor;
      constructor.run(geometry.m_sampling.begin(),geometry.m_sampling.end(),geometry.m_bvh);

      std::cout << "init_geometry(): Sample points = " << geometry.m_sampling.size() << std::endl;
      std::cout << "init_geometry(): BVH nodes = " << geometry.m_bvh.size() << std::endl;
    }

  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_INIT_GEOMETRY_H
#endif
