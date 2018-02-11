#ifndef OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_GEOMETRY_H
#define OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_GEOMETRY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/collision/aabb_tree/aabb_tree_triangle.h>
#include <OpenTissue/collision/bvh/bvh.h>

namespace OpenTissue
{
  namespace aabb_tree
  {

    template<
      typename real_type_
      , typename vertex_data_type_
    >
    class Geometry
    {
    public:

      typedef real_type_                                           real_type;
      typedef OpenTissue::math::BasicMathTypes<real_type,size_t>   math_types;
      typedef vertex_data_type_                                    vertex_data_type;
      typedef OpenTissue::geometry::AABB<math_types>               volume_type;

      typedef OpenTissue::aabb_tree::TriangleWrapper<vertex_data_type>             geometry_type;
      typedef OpenTissue::bvh::BoundingVolumeHierarchy<volume_type,geometry_type>  bvh_type;

    public:

      bvh_type         m_bvh;                 ///< The BVH data structure.

    };

  } // namespace aabb_tree
} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_GEOMETRY_H
#endif
