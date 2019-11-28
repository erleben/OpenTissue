#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_AABB_DATA_QUERY_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_AABB_DATA_QUERY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/spatial_hashing/spatial_hashing_query.h>

#include <list>
#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {

    template<typename hash_grid, typename collision_policy>
    class AABBDataQuery 
      : public Query< AABBDataQuery<hash_grid,collision_policy>, hash_grid, collision_policy >
    {
    protected:

      typedef typename hash_grid::cell_type     cell_type;
      typedef typename hash_grid::triplet_type  triplet_type;
      typedef typename hash_grid::point_type    point_type;
      typedef typename std::list<cell_type*>    cell_queue;

    public:

      template< typename data_iterator  >
      void first_pass(data_iterator begin,data_iterator end )
      {
        size_t stamp = 0;

        typename hash_grid::cell_iterator Cbegin = this->begin();
        typename hash_grid::cell_iterator Cend = this->end();
        for( typename hash_grid::cell_iterator cell = Cbegin; cell!=Cend; ++cell)
          cell->m_query_stamp = stamp;

        for(data_iterator data=begin; data!=end; ++data)
        {
          ++stamp;

          point_type min_corner = collision_policy::min_coord( *data ); //--- from collision policy
          point_type max_corner = collision_policy::max_coord( *data ); //--- from collision policy
          triplet_type m = hash_grid::get_triplet(min_corner);
          triplet_type M = hash_grid::get_triplet(max_corner);

          assert( m(0) <= M(0) || !"Minimum was larger than maximum");
          assert( m(1) <= M(1) || !"Minimum was larger than maximum");
          assert( m(2) <= M(2) || !"Minimum was larger than maximum");

          triplet_type triplet(m);
          for ( triplet(0)= m(0) ; triplet(0) <= M(0); ++triplet(0) )
            for ( triplet(1)= m(1) ; triplet(1) <= M(1); ++triplet(1) )
              for ( triplet(2)= m(2) ; triplet(2) <= M(2); ++triplet(2) )
              {
                cell_type & cell = hash_grid::get_cell(triplet);
                if(cell.m_query_stamp==stamp)
                  continue;
                cell.m_query_stamp=stamp;
                cell.add( *data );
              }
        }
      }

      template< typename data_iterator  >
      void remove_data(data_iterator begin,data_iterator end )
      {
        for(data_iterator data=begin; data!=end; ++data)
        {
          point_type min_corner = collision_policy::min_coord( *data ); //--- from collision policy
          point_type max_corner = collision_policy::max_coord( *data ); //--- from collision policy
          triplet_type m = hash_grid::get_triplet(min_corner);
          triplet_type M = hash_grid::get_triplet(max_corner);

          assert( m(0) <= M(0) || !"Minimum was larger than maximum");
          assert( m(1) <= M(1) || !"Minimum was larger than maximum");
          assert( m(2) <= M(2) || !"Minimum was larger than maximum");

          triplet_type triplet(m);
          for ( triplet(0)= m(0) ; triplet(0) <= M(0); ++triplet(0) )
            for ( triplet(1)= m(1) ; triplet(1) <= M(1); ++triplet(1) )
              for ( triplet(2)= m(2) ; triplet(2) <= M(2); ++triplet(2) )
              {
                cell_type & cell = hash_grid::get_cell(triplet);
                cell.remove( *data );
              }
        }
      }

    };

  } // namespace spatial_hashing
} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_AABB_DATA_QUERY_H
#endif
