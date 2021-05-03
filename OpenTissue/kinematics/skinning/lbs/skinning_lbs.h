#ifndef OPENTISSUE_KINEMATICS_SKINNING_LBS_SKINNING_LBS_H
#define OPENTISSUE_KINEMATICS_SKINNING_LBS_SKINNING_LBS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/trimesh/trimesh.h>
#include <OpenTissue/core/containers/mesh/trimesh/util/trimesh_util.h>
#include <OpenTissue/kinematics/skinning/skinning_traits.h>
#include <cassert>
#include <vector>


namespace OpenTissue
{
  namespace skinning
  {

    template<typename types>
    class LBS : public trimesh::TriMesh<types,SkinVertexTraits<types> ,SkinFaceTraits<types>, trimesh::TriMeshArrayKernel >
    {
    public:
      // Sort of a concept check to avoid that software skinning algorithms
      // uses gpu skinning features and vice versa
      typedef int	no_gpu_support;

      typedef typename types::matrix3x3_type		matrix3x3_type;
      typedef typename types::quaternion_type		quaternion_type;
      typedef typename types::vector3_type		  vector3_type;
      typedef typename types::real_type			    real_type;

      //TODO: Ugly hack to circumvent VC++ 8.0 name-lookup error. Without OpenTissue::, it gives C3200.
      typedef trimesh::TriMesh< types, SkinVertexTraits<types>, SkinFaceTraits<types>, OpenTissue::trimesh::TriMeshArrayKernel >  base_mesh;

    public:

      int m_material_idx;

    public:

      LBS()
        : m_material_idx(-1)
      {}

    public:

      bool empty() const 
      {   
        return (!(base_mesh::size_faces()>0));  
      }

      template<typename skin_container_type>
      static void init_skin_render( skin_container_type & scont )
      {
      }

      template<typename skin_container_type>
      static void cleanup_skin_render( skin_container_type & scont )
      {
      }

      template<typename skin_container_type>
      static void pre_render( skin_container_type & scont )
      {
      }

      template<typename skin_container_type>
      static void post_render( skin_container_type & scont )
      {
      }

      /**
      * Prepare skin for visualization.
      *
      * NOTE: A skin mesh is living in bone-space!
      *
      * @param skeleton     The skeleton containing the bone space transforms that
      *                     is used to place sking correctly in the world.
      */
      template<typename skeleton_type, typename skin_container_type>
      void update( skeleton_type & skeleton, skin_container_type & scont )
      {
        typedef typename skeleton_type::bone_traits   bone_traits;
        typedef typename types::coordsys_type         coordsys_type;

        //--- test if we need to do any work at all!!!
        if(this->empty())
          return;

        typedef typename skeleton_type::bone_type  bone_type;

        typename base_mesh::vertex_iterator vertex = base_mesh::vertex_begin();
        typename base_mesh::vertex_iterator end    = base_mesh::vertex_end();

        for(;vertex!=end;++vertex)
        {
          vertex->m_coord  = vector3_type(0,0,0);
          vertex->m_normal = vector3_type(0,0,0);

          for(int i= 0; i < vertex->m_influences; ++i)
          {
            vector3_type p = vertex->m_original_coord;
            vector3_type n = vertex->m_original_normal;
            bone_type const * bone = skeleton.get_bone(vertex->m_bone[i]);

            coordsys_type bone_space_transform = bone_traits::convert( bone->bone_space_transform() );

            bone_space_transform.xform_point(p);
            bone_space_transform.xform_vector(n);
            vertex->m_coord  += vertex->m_weight[i] * p;
            vertex->m_normal += vertex->m_weight[i] * n;
          }
        }
      }

    };

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_LBS_SKINNING_LBS_H
#endif
