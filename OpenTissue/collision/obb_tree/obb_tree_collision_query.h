#ifndef OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_COLLISION_QUEURY_H
#define OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_COLLISION_QUEURY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh_model_collision_query.h>

namespace OpenTissue
{

  namespace obb_tree
  {

    template <typename collision_policy>
    class CollisionQuery 
      : public OpenTissue::bvh::ModelCollisionQuery<collision_policy>
    {
    public:
      typedef OpenTissue::bvh::ModelCollisionQuery<collision_policy> base_type;
    public:

      CollisionQuery()
        : OpenTissue::bvh::ModelCollisionQuery<collision_policy>()
      {}

    public:

      template<typename coordsys_type,typename bvh_type, typename results_container>
      void run( coordsys_type const & A2B, bvh_type const & bvh_A, bvh_type const & bvh_B, results_container & results )
      {
        this->m_query = this->get_next_time_stamp(); //--- from collision policy
        if(bvh_A.size()<bvh_B.size())
          base_type::run(A2B,bvh_A,bvh_B,results);
        else
        {
          coordsys_type B2A = inverse(A2B);
          base_type::run(B2A,bvh_B,bvh_A,results);
        }
      }
    };

  } // namespace obb_tree

} // namespace OpenTissue

//OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_COLLISION_QUEURY_H
#endif
