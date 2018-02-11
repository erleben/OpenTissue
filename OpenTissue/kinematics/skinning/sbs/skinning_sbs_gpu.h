#ifndef OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_SBS_GPU_H
#define OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_SBS_GPU_H
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

// Shader program header includes
#include <OpenTissue/gpu/cg/cg_program.h>

namespace OpenTissue
{
  namespace skinning
  {

    template<typename types>
    class SBSGPU 
      : public trimesh::TriMesh<types,SkinVertexTraits<types> ,SkinFaceTraits<types>, trimesh::TriMeshArrayKernel >
    {
    public:
      // Sort of a concept check to avoid that software skinning algorithms
      // uses gpu skinning features and vice versa
      typedef int	gpu_support;

      typedef typename types::matrix3x3_type		matrix3x3_type;
      typedef typename types::quaternion_type		quaternion_type;
      typedef typename types::vector3_type		  vector3_type;
      typedef typename types::real_type			    real_type;

      //TODO: Ugly hack to circumvent VC++ 8.0 name-lookup error. Without OpenTissue::, it gives C3200.
      typedef trimesh::TriMesh<types, SkinVertexTraits<types>, SkinFaceTraits<types>, OpenTissue::trimesh::TriMeshArrayKernel >	base_mesh;

      //--- Shader program data
      typedef OpenTissue::cg::Program	Program;

    public:

      int	m_material_idx;

    public:

      SBSGPU()
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
        gl::gl_check_errors( "init_rendering: called" );

        // Startup CG
        OpenTissue::cg::startup();

        // Load the Cg program
        std::string filename = 
          opentissue_path + 
          "/OpenTissue/kinematics/skinning/SBS/sbs_skin_shader.cg";
        scont.m_vp.load_from_file( cg::Program::vertex_program, filename );
        gl::gl_check_errors( "init_fragment_program: program loaded" );
      }

      template<typename skin_container_type>
      static void cleanup_skin_render( skin_container_type & scont )
      {
        gl::gl_check_errors("cleanup_rendering: called");
        OpenTissue::cg::shutdown();
      }


      template<typename skin_container_type>
      static void pre_render( skin_container_type & scont )
      {
        scont.m_vp.set_modelview_projection_matrix();
        scont.m_vp.set_modelview_inverse_transpose();

        scont.m_vp.enable();

        scont.m_uploadBones = true;
        scont.m_uploadRc = true;

        // Clear lookup tables and lut for next frame...
        scont.m_rc_cached.clear();
        scont.m_rc_lut.clear();
      }

      template<typename skin_container_type>
      static void post_render( skin_container_type & scont )
      {
        scont.m_vp.disable();
      }

      /**
      * Prepare skin for visualization.
      *
      * NOTE: A skin mesh is living in bone-space!
      *
      * @param skeleton     The skeleton containing the bone space transforms that
      *                     is used to place sking correctly in the world.
      */

      // TODO: reset skeleton to const, i.e. use const iterators...
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

        if( scont.m_uploadBones )
        {
          // Allocate space for all bones
          float *allBones = new float[9*skeleton.size()]; // float4x4 has size=16
          unsigned int k=0;

          // Create array with float4x4 matrices, coloumn order
          typename skeleton_type::bone_iterator abone = skeleton.begin();
          typename skeleton_type::bone_iterator boneEnd = skeleton.end();

          for( ; abone!=boneEnd; ++abone )
          {
            coordsys_type bone_space_transform = bone_traits::convert( abone->bone_space_transform() );

            quaternion_type qt = bone_space_transform.Q();
            vector3_type trans = bone_space_transform.T();

            // Upload quaternions to the gpu
            allBones[0+k*9] = qt.v()(0);
            allBones[1+k*9] = qt.v()(1);
            allBones[2+k*9] = qt.v()(2);
            allBones[3+k*9] = qt.s();
            allBones[4+k*9] = 0;
            allBones[5+k*9] = 0;
            allBones[6+k*9] = trans( 0 );
            allBones[7+k*9] = trans( 1 );
            allBones[8+k*9] = trans( 2 );

            // Go to next matrix...
            ++k;
          }

          // Send bones to the gpu
          scont.m_vp.set_floatNxN_array_param( "bones", (int)skeleton.size(), allBones );

          // Delete bone array, they now reside on GPU :-)
          delete[] allBones;

          scont.m_uploadBones = false;
        }

        if( scont.m_uploadRc )
        {
          typename base_mesh::vertex_iterator vertex = base_mesh::vertex_begin();
          typename base_mesh::vertex_iterator end    = base_mesh::vertex_end();
          bone_container_type affecting_bones;

          for(;vertex!=end;++vertex)
          {
            if( vertex->m_influences > 1 )
            {
              // clear bones container
              affecting_bones.clear();

              vector3_type r_c( 0, 0, 0 );

              for(int i=0; i < vertex->m_influences; ++i)
              {
                bone_type const * bone = skeleton.get_bone( vertex->m_bone[i] );
                affecting_bones.push_back( bone );
              }

              //--- Lookup center of rotation
              typename lut_type::iterator lut_it = scont.m_rc_lut.find( vertex->m_key );
              if( lut_it != scont.m_rc_lut.end() )
                r_c = scont.m_rc_cached[lut_it->second];
              else
              {
                //--- Compute center of rotation
                r_c = center_of_rotation( skeleton, affecting_bones );

                //--- Cache values
                scont.m_rc_cached.push_back( r_c );
                scont.m_rc_lut[vertex->m_key] = scont.m_rc_cached.size()-1;
              }

            }
          }

          // We have all rotation centers - yay!
          if( scont.m_rc_lut.size() == scont.m_num_keys )
          {
            // Allocate space for all bones
            float *allRc = new float[3*scont.m_rc_lut.size()];

            typename lut_type::iterator lut_it = scont.m_rc_lut.begin();
            for( ; lut_it != scont.m_rc_lut.end(); ++lut_it )
            {
              size_t idx = lut_it->second;
              vector3_type r_c = scont.m_rc_cached[idx];

              allRc[3*idx]	= r_c(0);
              allRc[3*idx+1]	= r_c(1);
              allRc[3*idx+2]	= r_c(2);

            }

            // Send rotation centers to the gpu
            scont.m_vp.set_float3_array_param( "r_c", (int)scont.m_rc_lut.size(), allRc );

            delete[] allRc;

            scont.m_uploadRc = false;
          }
        }

      }
    };

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_SBS_SKINNING_SBS_GPU_H
#endif
