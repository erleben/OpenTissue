#ifndef OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_REFITTER_POLICY_H
#define OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_REFITTER_POLICY_H
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

  template<typename aabb_tree_geometry>
  class RefitterPolicy
  {
  public:

    typedef typename aabb_tree_geometry::bvh_type       bvh_type;
    typedef typename bvh_type::volume_type              volume_type;
    typedef typename bvh_type::geometry_type            geometry_type;
    typedef typename bvh_type::bv_ptr                   bv_ptr;
    typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;
    typedef typename bvh_type::annotated_bv_type        annotated_bv_type;
    typedef typename bvh_type::bv_iterator              bv_iterator;
    typedef typename bvh_type::geometry_iterator        geometry_iterator;
    typedef typename volume_type::real_type             real_type;
    typedef typename volume_type::vector3_type          vector3_type;

  public:

    real_type m_enlargement;   ///< Additive enlargement factor. All volumes are slightly
                               ///< enlarged to avoid problems with numerical precision and
                               ///< truncation. Default value is zero, but can be changed by user.

  public:

    RefitterPolicy()
      : m_enlargement(0)
    {}

  public:

    void refit(bv_ptr bv)
    {
      assert(bv);
      vector3_type & m = bv->volume().min();
      vector3_type & M = bv->volume().max();
      m = vector3_type( math::detail::highest<real_type>() );
      M = vector3_type( math::detail::lowest<real_type>() );
      if(bv->is_leaf())
      {
        annotated_bv_ptr annotated_bv = boost::static_pointer_cast<annotated_bv_type>(bv);

        geometry_type * geometry = &(*(annotated_bv->geometry_begin()));
        vector3_type p0 = geometry->m_p0->position();
        vector3_type p1 = geometry->m_p1->position();
        vector3_type p2 = geometry->m_p2->position();

        m = min(m, p0);
        m = min(m, p1);
        m = min(m, p2);
        M = max(M, p0);
        M = max(M, p1);
        M = max(M, p2);
        if(m_enlargement>0)
        {
          m -= vector3_type(m_enlargement,m_enlargement,m_enlargement);
          M += vector3_type(m_enlargement,m_enlargement,m_enlargement);
        }
      }
      else
      {
        for ( bv_iterator child = bv->child_begin();child!=bv->child_end();++child )
        {
          m = min( m,  child->volume().min() );
          M = max( M,  child->volume().max() );
        }
      }
    }
  };

} // namespace aabb_tree

} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_REFITTER_POLICY_H
#endif
