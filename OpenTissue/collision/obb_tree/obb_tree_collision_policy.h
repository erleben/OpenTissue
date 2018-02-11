#ifndef OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_COLLISION_POLICY_H
#define OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_COLLISION_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/intersect/intersect_obb_obb_sat.h>
#include <OpenTissue/collision/intersect/intersect_triangle_triangle_sat.h>

#include <cassert>

namespace OpenTissue
{

  namespace obb_tree
  {

    template<
        typename bvh_type_
      , typename obb_tree_types_
    >
    class CollisionPolicy
    {
    public:

      typedef          obb_tree_types_                    obb_tree_types;
      typedef typename obb_tree_types::coordsys_type      coordsys_type;
      typedef typename obb_tree_types::vector3_type       vector3_type;
      typedef typename obb_tree_types::time_stamp_type    time_stamp_type;

      typedef bvh_type_                                   bvh_type;
      typedef typename bvh_type::volume_type              volume_type;
      typedef typename bvh_type::bv_ptr                   bv_ptr;
      typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;
      typedef typename bvh_type::annotated_bv_type        annotated_bv_type;
      typedef typename bvh_type::geometry_type            face_ptr_type;

    public:

      time_stamp_type   m_query;    ///< Time stamp of collision query. Used to see if cached bounding volumes are valid.

    public:

      /**
       * Get Next Time Stamp.
       * This methods computes a new global unique time stamp. It should 
       * be invoked by caller whenever a new time-stamp is wanted. Notice
       * this implementation technique keeps everything in headers only!
       *
       * @return  The new time stamp.
       */
      static time_stamp_type  get_next_time_stamp()
      {
        static time_stamp_type next_stamp = 0;
        ++next_stamp;
        return next_stamp;
      }

    public:
      CollisionPolicy()
        : m_query()
      {}

    public:

      template<typename result_container>
      void reset(result_container & results)
      {
        results.clear();
      }

      bool overlap(coordsys_type const & A2B, bv_ptr bvA, bv_ptr bvB)
      {
        if(bvA->m_query!=m_query)
        {
          bvA->m_cached_volume = bvA->volume();


          //--- If boxes are given wrt. model frames A and B then
          //--- box A can be transformed into box B  local frame by
          //matrix3x3_type R_A2B = A2B.Q();
          //vector3_type   T_A2B = A2B;
          //matrix3x3_type R     = inv(R_B) * R_A2B * R_A;
          //vector3_type   T     = T_A + T_A2B + inv(R_B) T_B;


          //--- If boxes are stored in their parents box local frames, and the parenting BV test used A2B then we can compute a new A2B
          //---
          //---     If we descend to child C of box A
          //---
          //---        A2B = A2B C
          //---
          //---     If we descend to child C of box B
          //---
          //---        A2B = inv (B) A2B
          //---
          //---  However we have no notion of traversal in this method invocation!!


          bvA->m_cached_volume.xform(A2B);  //--- KE 2005-11-24: This is a Bug!!!
          bvA->m_query = m_query;
        }
        bool collision = !OpenTissue::intersect::obb_obb_sat(bvB->volume(),bvA->m_cached_volume);
        return collision;
      }

      template<typename result_container>
      bool report(coordsys_type const & A2B, bv_ptr bvA, bv_ptr bvB, result_container & results)
      {       
        annotated_bv_ptr  nodeA = boost::static_pointer_cast<annotated_bv_type>(bvA);
        annotated_bv_ptr  nodeB = boost::static_pointer_cast<annotated_bv_type>(bvB);

        face_ptr_type A = *(nodeA->geometry_begin());
        face_ptr_type B = *(nodeB->geometry_begin());

        vector3_type a0 = *(A->m_v0);
        A2B.xform_point(a0);
        vector3_type a1 = *(A->m_v1);
        A2B.xform_point(a1);
        vector3_type a2 = *(A->m_v2);
        A2B.xform_point(a2);

        bool collision = OpenTissue::intersect::triangle_triangle_sat(  a0, a1 ,a2 , *(B->m_v0), *(B->m_v1), *(B->m_v2) );

        if(!collision)
          return false;

        results.push_back( std::make_pair(A,B) );

        return true;
      }
    };

  } // namespace obb_tree

} // namespace OpenTissue

//OPENTISSUE_COLLISION_OBB_TREE_OBB_TREE_COLLISION_POLICY_H
#endif
