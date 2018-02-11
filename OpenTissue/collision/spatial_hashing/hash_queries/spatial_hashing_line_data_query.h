#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_LINE_DATA_QUERY_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_LINE_DATA_QUERY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/spatial_hashing/spatial_hashing_query.h>
#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {

    template<typename hash_grid, typename collision_policy>
    class LineDataQuery 
      : public Query< LineDataQuery<hash_grid,collision_policy>, hash_grid, collision_policy >
    {
    public:
      typedef typename hash_grid::triplet_type triplet_type;
      typedef typename hash_grid::cell_type cell_type;
      typedef typename hash_grid::point_type point_type;
      typedef typename point_type::value_type real_type;

      template< typename data_iterator  >
      void first_pass(data_iterator begin, data_iterator end )
      {
        real_type parameters[6];
        bool  indices[6];


        //--- clear cell query stamps, so we can use them to watch against hash-collisions
        size_t stamp = 0;
        typename hash_grid::cell_iterator Cbegin = this->begin();
        typename hash_grid::cell_iterator Cend = this->end();
        for( typename hash_grid::cell_iterator cell = Cbegin;cell!=Cend;++cell)
          cell->m_query_stamp = stamp;

        real_type dx = this->get_spacing();

        for(data_iterator data = begin;data!=end;++data)
        {
          ++stamp;

          point_type o = origin(*data);
          point_type d = destination(*data);
          point_type u = d - o;
          real_type s = 0;
          triplet_type T = get_triplet(o);

          while (s<=1)
          {
            cell_type & cell = get_cell(T);
            if(cell.m_query_stamp!=stamp)
            {
              cell.m_query_stamp=stamp;
              cell.add( *data );
            }
            update_parameters(T,dx,o,u,parameters);
            s = find_indices(parameters,s,indices);
            update_triplet(T,indices);
          }
        }
      }


      template< typename data_iterator  >
      void remove_data(data_iterator begin, data_iterator end )
      {
        real_type parameters[6];
        bool  indices[6];


        //--- clear cell query stamps, so we can use them to watch against hash-collisions
        real_type dx = this->get_spacing();
        for(data_iterator data = begin;data!=end;++data)
        {
          point_type o = origin(*data);
          point_type d = destination(*data);
          point_type u = d - o;
          real_type s = 0;
          triplet_type T = get_triplet(o);

          while (s<=1)
          {
            cell_type & cell = get_cell(T);
            cell.remove( *data );
            update_parameters(T,dx,o,u,parameters);
            s = find_indices(parameters,s,indices);
            update_triplet(T,indices);
          }
        }
      }

    protected:

      void update_triplet(triplet_type & t,bool * indices)
      {
        if(indices[0])  t(0) -= 1;
        if(indices[1])  t(0) += 1;
        if(indices[2])  t(1) -= 1;
        if(indices[3])  t(1) += 1;
        if(indices[4])  t(2) -= 1;
        if(indices[5])  t(2) += 1;
      }

      void update_parameters(triplet_type & t,real_type dx,point_type const & o,point_type const & u,real_type * param)
      {
        if(u(0)!=0)
        {
          param[0] = (  t(0)*dx    - o(0)) / u(0);
          param[1] = ( (t(0)+1)*dx - o(0)) / u(0);
        }
        else
        {
          param[0] = 10e30;
          param[1] = 10e30;
        }
        if(u(1)!=0)
        {
          param[2] = (  t(1)*dx    - o(1)) / u(1);
          param[3] = ( (t(1)+1)*dx - o(1)) / u(1);
        }
        else
        {
          param[2] = 10e30;
          param[3] = 10e30;
        }
        if(u(2)!=0)
        {
          param[4] = (  t(2)*dx    - o(2)) / u(2);
          param[5] = ( (t(2)+1)*dx - o(2)) / u(2);
        }
        else
        {
          param[4] = 10e30;
          param[5] = 10e30;
        }
      }

      real_type find_indices(real_type * param,real_type s,bool * indices)
      {
        real_type min_val = 10e30;
        for(int i = 0;i<6;++i)
        {
          if(param[i]>s && param[i]<min_val)
            min_val = param[i];
        }
        for(int i = 0;i<6;++i)
        {
          if(param[i]==min_val)
            indices[i] = true;
          else
            indices[i] = false;
        }
        return min_val;
      }

    };
  } // namespace spatial_hashing

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_HASH_QUERIES_SPATIAL_HASHING_LINE_DATA_QUERY_H
#endif
