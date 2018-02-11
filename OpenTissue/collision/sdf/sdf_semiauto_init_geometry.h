#ifndef OPENTISSUE_COLLISION_SDF_SDF_SEMIAUTO_INIT_GEOMETRY_H
#define OPENTISSUE_COLLISION_SDF_SDF_SEMIAUTO_INIT_GEOMETRY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/sdf/sdf_init_geometry.h>
#include <OpenTissue/core/containers/grid/util/grid_mesh2phi.h>

namespace OpenTissue
{

  namespace sdf
  {


    /**
    * Initialize signed distance field geometry.
    *
    * This function is usefull if one do not have a pre-computed signed
    * distance field. The function tries to find a reasonable resolution
    * of a signed distance field and uses t4 cpu/gpu scan to compute the signed
    * distance field.
    *
    * @param mesh              The surface mesh (corresponds to the zero-level set of the signed distance field).
    * @param edge_resolution   Threshold value, indicating the sampling
    *                          resolution along edges. If zero it will be
    *                          computed on the fly, to match the resolution
    *                          of the signed distance field.
    *
    * @param face_sampling     Boolean flag indicating whether face sampling is on or off.
    * @param geometry          Upon return this argument holds the signed distance field geometry.
    * @param max_resolution    This argument can be used to set a maximum upper limit on the resolution of the signed distance field. Default value is 64.
    * @param use_gpu           Boolean flag indicating whether the gpu should be used to compute the signed distance field. Default value is true.
    */
    template<typename mesh_type,typename sdf_geometry_type>
    void semiauto_init_geometry(
      mesh_type /*const*/ & mesh
      , double edge_resolution
      , bool face_sampling
      , sdf_geometry_type & geometry
      , unsigned int max_resolution = 64
      , bool use_gpu = true
      )
    {
      typedef typename sdf_geometry_type::grid_type                        grid_type;

      grid_type phi;
      OpenTissue::grid::mesh2phi(mesh, phi, max_resolution, use_gpu);
      init_geometry(mesh,phi,edge_resolution,face_sampling,geometry);
    }


    /**
    * Initialize signed distance field geometry.
    *
    * This function is usefull if one do not have a pre-computed signed
    * distance field. The function used a pre-specified resolution and
    * band-size and uses t4 gpu/cpu scan to compute the signed
    * distance field.
    *
    * @param mesh              The surface mesh (corresponds to the zero-level set of the signed distance field).
    * @param edge_resolution   Threshold value, indicating the sampling
    *                          resolution along edges. If zero it will be
    *                          computed on the fly, to match the resolution
    *                          of the signed distance field.
    *
    * @param face_sampling     Boolean flag indicating whether face sampling is on or off.
    * @param geometry          Upon return this argument holds the signed distance field geometry.
    * @param bandsize          This argument can be used to set the size of a band enclosing the mesh.
    * @param resolution        This argument can be used to set the wanted resolution of the resuling distance field.
    * @param use_gpu           Boolean flag indicating whether the gpu should be used to compute the signed distance field. Default value is true.
    */
    template<typename mesh_type,typename sdf_geometry_type>
    void semiauto_init_geometry(
      mesh_type /*const*/ & mesh
      , double edge_resolution
      , bool face_sampling
      , sdf_geometry_type & geometry
      , double bandsize
      , unsigned int resolution
      , bool use_gpu = true
      )
    {
      typedef typename sdf_geometry_type::grid_type                        grid_type;

      grid_type phi;
      OpenTissue::grid::mesh2phi(mesh,phi, bandsize, resolution, use_gpu);
      init_geometry(mesh,phi,edge_resolution,face_sampling,geometry);
    }

  } // namespace sdf

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SDF_SDF_SEMIAUTO_INIT_GEOMETRY_H
#endif
