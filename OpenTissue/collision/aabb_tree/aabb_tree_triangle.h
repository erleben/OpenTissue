#ifndef OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_TRIANGLE_H
#define OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_TRIANGLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace aabb_tree
  {

    template<typename vertex_data_type>
    class TriangleWrapper
    {
    public:

      vertex_data_type * m_p0;
      vertex_data_type * m_p1;
      vertex_data_type * m_p2;

    };

  } // namespace aabb_tree
} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_AABB_TREE_TRIANGLE_H
#endif
