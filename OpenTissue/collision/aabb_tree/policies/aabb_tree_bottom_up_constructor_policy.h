#ifndef OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_BOTTOM_UP_CONSTRUCTOR_POLICY_H
#define OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_BOTTOM_UP_CONSTRUCTOR_POLICY_H
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
    class BottomUpConstructorPolicy 
      : public OpenTissue::bvh::BinaryMatchBottomUpPolicy<typename aabb_tree_geometry::bvh_type>
    {
    public:

      typedef typename aabb_tree_geometry::bvh_type       bvh_type;
      typedef OpenTissue::bvh::BVHGraph<bvh_type>              graph_type;
      typedef typename graph_type::node_ptr_type          node_ptr;
      typedef typename graph_type::edge_ptr_type          edge_ptr;
      typedef typename graph_type::edge_iterator          edge_iterator;
      typedef typename graph_type::node_iterator          node_iterator;
      typedef typename bvh_type::bv_type                  bv_type;
      typedef typename bvh_type::bv_iterator              bv_iterator;
      typedef typename bvh_type::annotated_bv_type        annotated_bv_type;
      typedef typename bvh_type::volume_type              volume_type;
      typedef typename bvh_type::geometry_type            geometry_type;
      typedef typename bvh_type::geometry_iterator        geometry_iterator;
      typedef typename volume_type::real_type             real_type;
      typedef typename volume_type::vector3_type          vector3_type;

    public:

      BottomUpConstructorPolicy() {}
      virtual ~BottomUpConstructorPolicy() {}

    protected:

      const unsigned int degree() const { return 8; }

    public:

      template<typename geometry_iterator,typename volume_iterator>
      volume_type fit(geometry_iterator g0,geometry_iterator g1,volume_iterator v0,volume_iterator v1)
      {
        unsigned int  vN = std::distance(v0,v1);
        if(vN>1)
          return fit_volumes(v0,v1);
        return fit_geometry(g0,g1);
      }

    private:

      template<typename geometry_iterator>
      volume_type fit_geometry( geometry_iterator begin,geometry_iterator end )
      {
        vector3_type m = vector3_type( math::detail::highest<real_type>() );
        vector3_type M = vector3_type( math::detail::lowest<real_type>() );
        for ( geometry_iterator geometry = begin;geometry!=end;++geometry)
        {
          vector3_type p0 = geometry->m_p0->position();
          vector3_type p1 = geometry->m_p1->position();
          vector3_type p2 = geometry->m_p2->position();

          m = min(m, p0);
          m = min(m, p1);
          m = min(m, p2);
          M = max(M, p0);
          M = max(M, p1);
          M = max(M, p2);
        }
        return volume_type(m,M);
      }

      template<typename volume_iterator>
      volume_type fit_volumes( volume_iterator begin, volume_iterator end )
      {
        vector3_type m = vector3_type( math::detail::highest<real_type>() );
        vector3_type M = vector3_type( math::detail::lowest<real_type>() );
        for ( volume_iterator volume = begin;volume!=end;++volume )
        {
          m = min( m,  volume->min() );
          M = max( M,  volume->max() );
        }
        return volume_type(m,M);
      }

    };

  } // namespace aabb_tree

} // namespace OpenTissue

// OPENTISSUE_COLLISION_AABB_TREE_POLICIES_AABB_TREE_BOTTOM_UP_CONSTRUCTOR_POLICY_H
#endif
