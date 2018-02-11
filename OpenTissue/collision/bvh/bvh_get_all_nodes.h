#ifndef OPENTISSUE_COLLISION_BVH_BVH_GET_ALL_NODES_H
#define OPENTISSUE_COLLISION_BVH_BVH_GET_ALL_NODES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp> // needed for  boost::const_pointer_cast

namespace OpenTissue
{
  namespace bvh
  {
    /**
    *
    */
    template<typename bvh_type,typename bv_ptr_container>
    inline void get_all_nodes(bvh_type const & bvh,bv_ptr_container & nodes)
    {
      typedef typename bvh_type::bv_type             bv_type;
      typedef typename bvh_type::bv_ptr              bv_ptr;
      typedef typename bvh_type::bv_ptr_iterator     bv_ptr_iterator;

      nodes.clear();

      if ( !bvh.root() )
        return ;

      bv_ptr root = boost::const_pointer_cast<bv_type>( bvh.root() );

      nodes.push_back( root );
      bv_ptr_container Q;
      Q.push_back( root );

      while ( !Q.empty() )
      {
        bv_ptr  bv( Q.front() ); Q.pop_front();
        bv_ptr_iterator child = bv->child_ptr_begin();
        bv_ptr_iterator end   = bv->child_ptr_end();
        for(;child!=end;++child)
        {
          bv_ptr ptr( *child );
          nodes.push_back( ptr );
          Q.push_back( ptr );
        }
      }
    }

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BVH_GET_ALL_NODES_H
#endif
