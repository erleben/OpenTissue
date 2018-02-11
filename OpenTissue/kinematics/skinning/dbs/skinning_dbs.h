#ifndef OPENTISSUE_KINEMATICS_SKINNING_QBS_SKINNING_QBS_H
#define OPENTISSUE_KINEMATICS_SKINNING_QBS_SKINNING_QBS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2010 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/trimesh/trimesh.h>
#include <OpenTissue/core/containers/mesh/trimesh/util/trimesh_util.h>

#include <OpenTissue/core/math/math_dual_quaternion.h>
#include <OpenTissue/kinematics/skinning/skinning_traits.h>

#include <OpenTissue/utility/gl/gl_draw_mesh.h>

#include <vector>

/**
 * @author Per B. Nielsen <per.b.nielsen@gmail.com>
 *
 * @section DESCRIPTION
 *
 * This is an implementation of Dual Quaternion Blend Skinning as described in the paper,
 * "Geometric Skinning with Approximate Dual Quaternion Blending" from 2008 by L. Kavan et. al.
 *
 * The implementation used the implementation of linear blend skinning as a starting point. My contribution was to
 * modify the update() function to use dual quaternion blending.
 */
namespace OpenTissue
{
  namespace skinning
  {

    template<typename types>
    class DBS : public trimesh::TriMesh<types, SkinVertexTraits<types>, SkinFaceTraits<types>, trimesh::TriMeshArrayKernel >
    {
    protected:

      typedef typename types::value_traits            value_traits;

    public:
      // Sort of a concept check to avoid that software skinning algorithms
      // uses gpu skinning features and vice versa
      typedef int      no_gpu_support;


      typedef typename types::matrix3x3_type          matrix3x3_type;
      typedef typename types::quaternion_type         quaternion_type;
      typedef typename types::dual_quaternion_type    dual_quaternion_type;
      typedef typename types::vector3_type            vector3_type;
      typedef typename types::real_type               real_type;

      // 2010-07-16 Kenny: code review, rewrite to proper review comment
      // 2010-07-16 perb: This is taken straight out of skinning_(sbs|lbs).h
      
      //TODO: Ugly hack to circumvent VC++ 8.0 name-lookup error. Without OpenTissue::, it gives C3200.
      typedef trimesh::TriMesh< types, SkinVertexTraits<types>, SkinFaceTraits<types>, OpenTissue::trimesh::TriMeshArrayKernel >  base_mesh;


    public:

      int m_material_idx;

    public:

      DBS()
        : m_material_idx( -1 )
      {}

    public:

      bool empty() const
      {
        return ( !( base_mesh::size_faces() > 0 ) );
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
      *                     is used to place skin correctly in the world.
      */
      template< typename skeleton_type, typename skin_container_type >
      void update( skeleton_type & skeleton, skin_container_type & scont )
      {
        typedef typename skeleton_type::bone_traits   bone_traits;
        typedef typename types::coordsys_type         coordsys_type;

        //--- test if we need to do any work at all!!!
        if ( this->empty() ) return;

// 2010-07-16 Kenny: code review, is this really efficient? if a skeleton got multiple skin parts, then this skeleton conversion is done for each skin part rather than being shared by all skin parts?
// 2010-07-16 perb: Well no, as I note below, this is not efficient. I would prefer to let the skeleton save the bones as dual quaternions.
        
        // Transform all bones to dual quaternions
        // TODO: Let the skeleton save the bones as dual quaternions, so we don't have to convert them every frame!
        //       The time to convert to dual quaternions is approximately 1/10 of the total time for skinning on the test model
        std::vector< dual_quaternion_type > dq( skeleton.size() );

        for ( typename skeleton_type::const_bone_iterator bone = skeleton.begin() ; bone != skeleton.end() ; ++bone )
        {
          coordsys_type bone_space_transform = bone_traits::convert( bone->bone_space_transform() );

          dq[ bone->get_number() ] = dual_quaternion_type( bone_space_transform.Q(), bone_space_transform.T() );
        }

        // Perform linear blending
        typename base_mesh::vertex_iterator vertex = base_mesh::vertex_begin();
        typename base_mesh::vertex_iterator end    = base_mesh::vertex_end();

        // Implementation of "Geometric Skinning with Approximate Dual Quaternion Blending" - Algorithm 1
        for ( ; vertex != end ; ++vertex )
        {
          // Sum all influences
          dual_quaternion_type b = vertex->m_weight[0] * dq[ vertex->m_bone[0] ];
          quaternion_type bone0 = dq[ vertex->m_bone[0] ].r();

          for ( int i = 1; i < vertex->m_influences; ++i )
          {
            dual_quaternion_type current = dq[ vertex->m_bone[i] ];

            // Deal with antipodality
            if ( bone0 * current.r() < value_traits::zero() ) current = -current;

            b += vertex->m_weight[i] * current;
          }

          real_type b_0_norm = length( b.r() );

          quaternion_type c_0 = b.r() / b_0_norm;
          quaternion_type c_e = b.d() / b_0_norm;

          // Apply transformation to the coordinate
          vector3_type v = vertex->m_original_coord;

          vertex->m_coord = v + value_traits::two() * cross( c_0.v(), cross( c_0.v(), v ) + c_0.s() * v );
          vertex->m_coord += value_traits::two() * ( c_0.s() * c_e.v() - c_e.s() * c_0.v() + cross( c_0.v(), c_e.v() ) );

          // Apply transformation to the normal
          vector3_type n = vertex->m_original_normal;
          
          vertex->m_normal = n + value_traits::two() * cross( c_0.v(), cross( c_0.v(), n ) + c_0.s() * n );
        }
      }
    };
  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_QBS_SKINNING_QBS_H
#endif
