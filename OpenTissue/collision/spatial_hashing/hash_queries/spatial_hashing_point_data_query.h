#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_POINT_DATA_QUERY_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_POINT_DATA_QUERY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/spatial_hashing/spatial_hashing_query.h>

namespace OpenTissue
{
  namespace spatial_hashing
  {

    template<typename hash_grid, typename collision_policy>
    class PointDataQuery 
      : public Query< PointDataQuery<hash_grid,collision_policy>, hash_grid, collision_policy >
    {
    public:

      template< typename data_iterator  >
      void first_pass(data_iterator begin,data_iterator end )
      {
        for(data_iterator data=begin;data!=end;++data)
        {
          typename hash_grid::point_type p = Query< PointDataQuery<hash_grid,collision_policy>, hash_grid, collision_policy >::position(*data);
          Query< PointDataQuery<hash_grid,collision_policy>, hash_grid, collision_policy >::get_cell(p).add( *data );
        }
      }


      template< typename data_iterator  >
      void remove_data(data_iterator begin,data_iterator end )
      {
        for(data_iterator data=begin;data!=end;++data)
        {
          typename hash_grid::point_type p = Query< PointDataQuery<hash_grid,collision_policy>, hash_grid, collision_policy >::position(*data);
          Query< PointDataQuery<hash_grid,collision_policy>, hash_grid, collision_policy >::get_cell(p).remove( *data );
        }
      }

    };

  } // namespace spatial_hashing

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_POINT_DATA_QUERY_H
#endif
