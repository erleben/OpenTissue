#ifndef OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_HEIGHT_H
#define OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_HEIGHT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp> //needed for boost::const_pointer_cast

#include <map>
#include <list>

namespace OpenTissue
{
  namespace bvh
  {

    namespace detail
    {

      template<typename bvh_type, typename bv_ptr_type, typename bv_ptr_container>
      inline unsigned int visit(bvh_type const & bvh, bv_ptr_type bv, unsigned int const & height, bv_ptr_container & nodes )
      {
        typedef typename bvh_type::bv_ptr_iterator   bv_ptr_iterator;

        using std::max;

        unsigned int bv_height = 0;

        bv_ptr_iterator child = bv->child_ptr_begin();
        bv_ptr_iterator   end = bv->child_ptr_end();
        for(;child!=end;++child)
        {
          bv_ptr_type ptr( *child );
          bv_height = max( bv_height, visit(bvh, ptr, height, nodes ) );
        }
        bv_height = bv_height + 1;
        if ( bv_height == height )
        {
          nodes.push_back( bv );
        }
        return bv_height;
      }

    } // namespace detail

    /**
    *
    */
    template<typename bvh_type,typename bv_ptr_container>
    inline void get_nodes_at_height(bvh_type const & bvh,unsigned int const & height,bv_ptr_container & nodes)
    {
      typedef typename bvh_type::bv_ptr   bv_ptr;
      typedef typename bvh_type::bv_type  bv_type;

      nodes.clear();
      if(!bvh.root())
        return;
      bv_ptr root = boost::const_pointer_cast<bv_type>( bvh.root() );

      detail::visit( bvh, root, height, nodes );
    }

  } // namespace bvh


} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_HEIGHT_H
#endif
