#ifndef OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_CLOSEST_HEIGHT_H
#define OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_CLOSEST_HEIGHT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh_get_all_nodes.h>

#include <boost/shared_ptr.hpp> //needed for boost::const_pointer_cast

#include <map>

namespace OpenTissue
{
  namespace bvh
  {
    namespace detail
    {

      /**
       *
       */
      template<typename bvh_type, typename bv_ptr,typename height_map>
      inline unsigned int compute_heights(bvh_type const & bvh, bv_ptr bv, height_map & heights )
      {
        typedef typename bvh_type::bv_ptr_iterator       bv_ptr_iterator;
        typedef typename bvh_type::bv_type               bv_type;

        using std::max;

        unsigned int bv_height = 0;
        bv_ptr_iterator child = bv->child_ptr_begin();
        bv_ptr_iterator end   = bv->child_ptr_end();
        for( ;child != end; ++child)
        {
          bv_ptr ptr( *child );
          bv_height = max( bv_height, compute_heights(bvh, ptr, heights ) );
        }
        bv_height = bv_height + 1;
        heights[ bv ] = bv_height;
        return bv_height;
      }

    } // namespace detail


    /**
    *
    */
    template<typename bvh_type, typename bv_ptr_container>
    inline void get_nodes_at_closest_height(bvh_type const & bvh,unsigned int const & height,bv_ptr_container & nodes)
    {
      typedef typename bvh_type::bv_ptr                 bv_ptr;
      typedef typename bvh_type::bv_type                bv_type;
      typedef typename bvh_type::bv_ptr_container       internal_bv_ptr_container;
      typedef typename bvh_type::bv_ptr_iterator        bv_ptr_iterator;
      typedef typename std::map< bv_ptr, unsigned int>  height_map;

      nodes.clear();
      if(!bvh.root())
        return;

      height_map heights;

      bv_ptr root = boost::const_pointer_cast<bv_type>( bvh.root() );

      detail::compute_heights( bvh, root, heights );

      internal_bv_ptr_container all;
      bvh::get_all_nodes(bvh,all);
      bv_ptr_iterator bv  = all.begin();
      bv_ptr_iterator end = all.end();

      for(; bv!=end; ++bv )
      {
        bv_ptr cur( *bv );
        unsigned int bv_height = heights[ cur ];
        if ( bv_height == height )
          nodes.push_back( cur );
        else if ( (*bv)->parent() )
        {
          bv_ptr ptr = boost::const_pointer_cast<bv_type>( (*bv)->parent() );
          bv_ptr parent( ptr );
          unsigned int parent_height= heights[ parent ];
          if ( bv_height < height && parent_height > height )
            nodes.push_back( cur );
        }
      }
    }
  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BVH_GET_NODES_AT_CLOSEST_HEIGHT_H
#endif
