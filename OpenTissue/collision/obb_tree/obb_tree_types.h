#ifndef OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_TYPES_H
#define OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_TYPES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/collision/bvh/bvh_bounding_volume_hierarchy.h>
#include <OpenTissue/collision/bvh/top_down_constructor/bvh_top_down_constructor.h>

#include <OpenTissue/core/geometry/geometry_obb.h>
#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/math/math_basic_types.h>

#include <OpenTissue/collision/obb_tree/obb_tree_collision_policy.h>
#include <OpenTissue/collision/obb_tree/obb_tree_collision_query.h>
#include <OpenTissue/collision/obb_tree/obb_tree_top_down_policy.h>

#include <list>

namespace OpenTissue
{
  namespace obb_tree
  {

    template<
      typename math_types_ = math::BasicMathTypes<double,size_t>
    >
    class OBBTreeTypes 
      : public math_types_
    {

    public:

      typedef          OBBTreeTypes<math_types_>                obb_tree_types;
      typedef          math_types_                              math_types;
      typedef typename math_types::index_type                   time_stamp_type;
      typedef          OpenTissue::geometry::OBB<math_types>    obb_type;
      typedef          OpenTissue::geometry::Plane<math_types>  plane_type;

    protected:

      template<typename MT>
      class OBBTreeVertexTraits : public mesh::DefaultVertexTraits<MT>
      {};

      class OBBTreeHalfEdgeTraits : public mesh::DefaultHalfEdgeTraits
      {};

      class OBBTreeEdgeTraits : public mesh::DefaultEdgeTraits
      {};

      class OBBTreeMeshTraits : public mesh::DefaultMeshTraits
      {};


      template<typename MT>
      class OBBTreeFaceTraits : public mesh::DefaultFaceTraits
      {
      protected:

        typedef typename MT::vector3_type  vector3_type;
      public:

        vector3_type * m_v0;
        vector3_type * m_v1;
        vector3_type * m_v2;
        vector3_type * m_n0;
        vector3_type * m_n1;
        vector3_type * m_n2;

      public:

        OBBTreeFaceTraits()
          : m_v0()
          , m_v1()
          , m_v2()
          , m_n0()
          , m_n1()
          , m_n2()
        {}


      };

      template<typename MT>
      class OBBTreeMesh
        : public polymesh::PolyMesh<
          MT
        , OBBTreeVertexTraits<MT>
        , OBBTreeHalfEdgeTraits
        , OBBTreeEdgeTraits
        , OBBTreeFaceTraits<MT>
        , OBBTreeMeshTraits
        , polymesh::PolyMeshListKernel
        >
      {};

    public:

      typedef OBBTreeMesh< math_types >         mesh_type;
      typedef typename mesh_type::face_type *   face_ptr_type;

    protected:

      class OBBTreeBVTraits
      {
      public:

        time_stamp_type   m_query;             ///< Collision query time stamp. Used to verify whether cached volume is valid or not.
        obb_type          m_cached_volume;     ///< Cached model update transform of BV volume.

      public:

        OBBTreeBVTraits()
          : m_query()
          , m_cached_volume()
        {}

      };


    public:

      typedef OpenTissue::bvh::BoundingVolumeHierarchy<obb_type,face_ptr_type,OBBTreeBVTraits>  bvh_type;

    protected:

      typedef TopDownPolicy<bvh_type,obb_tree_types>  top_down_type;
      typedef CollisionPolicy<bvh_type,obb_tree_types> collision_type;

    public:

      typedef OpenTissue::bvh::TopDownConstructor< bvh_type,  top_down_type >  construtor_type;
      typedef CollisionQuery< collision_type >                                 collision_query_type;
      typedef std::list< std::pair<face_ptr_type,face_ptr_type> >              result_type;

    };

  }//namespace obb_tree

} // namespace OpenTissue

//OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_TYPES_H
#endif
