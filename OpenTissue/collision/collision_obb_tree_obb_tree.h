#ifndef OPENTISSUE_COLLISION_COLLISION_OBB_TREE_OBB_TREE_H
#define OPENTISSUE_COLLISION_COLLISION_OBB_TREE_OBB_TREE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/obb_tree/obb_tree_types.h>

namespace OpenTissue
{
  namespace collision
  {

    template<typename obb_tree_types>
    bool obb_tree_obb_tree(
      typename obb_tree_types::coordsys_type const & Awcs
      , typename obb_tree_types::bvh_type const & A
      , typename obb_tree_types::coordsys_type const & Bwcs
      , typename obb_tree_types::bvh_type const & B
      , typename obb_tree_types::result_type & results
      )
    {
      typedef typename obb_tree_types::construtor_type        contructor_type;
      typedef typename obb_tree_types::coordsys_type          coordsys_type;
      typedef typename obb_tree_types::collision_query_type   query_type;

      coordsys_type A2B = OpenTissue::math::model_update(Awcs,Bwcs);
      query_type query;
      query.run(A2B,A,B,results);
      return (results.size()>0);

    }

  }// namespace collision

} // namespace OpenTissue

//OPENTISSUE_COLLISION_COLLISION_OBB_TREE_OBB_TREE_H
#endif
