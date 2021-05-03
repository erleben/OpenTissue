#ifndef OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_SBS_H
#define OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_SBS_H
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

#include <OpenTissue/kinematics/skinning/sbs/skinning_center_of_rotation.h>
#include <OpenTissue/kinematics/skinning/skinning_traits.h>

#include <cassert>
#include <vector>

// Spherical Blend Skinning by Kasper A. Andersen @ DIKU, spreak@spreak.dk

namespace OpenTissue
{
  namespace skinning
  {


    template<typename types>
    class SBS : public trimesh::TriMesh<types,SkinVertexTraits<types> ,SkinFaceTraits<types>, trimesh::TriMeshArrayKernel >
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
      typedef trimesh::TriMesh<types, SkinVertexTraits<types>, SkinFaceTraits<types>, OpenTissue::trimesh::TriMeshArrayKernel > base_mesh;

    public:
      int	m_material_idx;

    public:

      SBS()
        : m_material_idx(-1)
      {}

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
        scont.m_rc_cached.clear();
        scont.m_rc_lut.clear();
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

        typedef typename skin_container_type::lut_type	lut_type;
        typedef typename skeleton_type::bone_type		bone_type;
        typedef std::vector<bone_type const *>			bone_container_type;

        typename base_mesh::vertex_iterator vertex = base_mesh::vertex_begin();
        typename base_mesh::vertex_iterator end    = base_mesh::vertex_end();
        bone_container_type affecting_bones;

        for(;vertex!=end;++vertex)
        {
          vertex->m_coord  = vector3_type(0,0,0);
          vertex->m_normal = vector3_type(0,0,0);

          //--- Now, use sbs to compute deform the mesh

          if( vertex->m_influences > 1 )
          {
            matrix3x3_type Q;
            vector3_type r_c( 0, 0, 0 );
            vector3_type r_cTrans( 0, 0, 0 );


            // clear bones container
            affecting_bones.clear();


            //--- Choose pivot quaternion
            coordsys_type bone_space_transform = bone_traits::convert( skeleton.get_bone( vertex->m_bone[0] )->bone_space_transform() );

            quaternion_type pivot = bone_space_transform.Q();
            affecting_bones.push_back( skeleton.get_bone( vertex->m_bone[0] ) );
            quaternion_type Qlerp = vertex->m_weight[0] * pivot;

            //--- Collect all affecting bones in a container so a center of rotation
            //--- can be computed...
            //--- Compute the rotation part, Q, in the progress
            for(int i= 1; i < vertex->m_influences; ++i)
            {
              bone_type const * bone = skeleton.get_bone( vertex->m_bone[i] );
              affecting_bones.push_back( bone );

              coordsys_type bone_space_transform = bone_traits::convert( bone->bone_space_transform() );

              quaternion_type q_ji = bone_space_transform.Q();

              //--- Check if quaternion should be reversed
              real_type norm = pivot * q_ji;
              if( norm < 0)
                q_ji = -q_ji;

              Qlerp += vertex->m_weight[i] * q_ji;
            }
            Q = normalize( Qlerp );

            //--- Lookup center of rotation
            typename lut_type::iterator lut_it = scont.m_rc_lut.find( vertex->m_key );
            if( lut_it != scont.m_rc_lut.end() )
            {
              r_c = scont.m_rc_cached[lut_it->second];
            }
            else
            {
              //--- Compute center of rotation
              r_c = center_of_rotation( skeleton, affecting_bones );

              //--- Cache values
              scont.m_rc_cached.push_back( r_c );
              scont.m_rc_lut[vertex->m_key] = scont.m_rc_cached.size()-1;
            }

            //--- Compute translation part, r_cTrans
            for(int i= 0; i < vertex->m_influences; ++i)
            {
              vector3_type p = r_c;
              vector3_type n = vertex->m_original_normal;
              bone_type const * bone = skeleton.get_bone(vertex->m_bone[i]);

              coordsys_type bone_space_transform = bone_traits::convert( bone->bone_space_transform() );

              //--- r_c handling
              bone_space_transform.xform_point( p );
              r_cTrans  += vertex->m_weight[i] * p;

              //--- Normal handling
              bone_space_transform.xform_vector( n );
              vertex->m_normal += vertex->m_weight[i] * n;
            }

            //--- Finally, the 'correct' deformation of the vertex can be computed
            vector3_type vr_c = vertex->m_original_coord - r_c; 
            vertex->m_coord = (Q * vr_c) + r_cTrans;
          }
          else
          {
            //--- Just one influence, pretty straight-forward...
            vertex->m_coord  = vertex->m_original_coord;
            vertex->m_normal = vertex->m_original_normal;
            bone_type const * bone = skeleton.get_bone(vertex->m_bone[0]);

            coordsys_type bone_space_transform = bone_traits::convert( bone->bone_space_transform() );
            
            bone_space_transform.xform_point( vertex->m_coord );
            bone_space_transform.xform_vector( vertex->m_normal );
          }
        }
      }

    };

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_SBS_H
#endif
