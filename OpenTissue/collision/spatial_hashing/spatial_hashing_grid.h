#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_SPATIAL_HASHING_GRID_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_SPATIAL_HASHING_GRID_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/iterator/indirect_iterator.hpp>

#include <vector>
#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {

    /**
    * Hash Grid.
    * This class stores an infinite uniform 3D grid as a 1D hash tabel. It is tailored to
    * work in 3D. However, an end user could use it in 2D or even 1D by ignoring second
    * or third coordinates (i.e. set them all to the same value).
    *
    * The hash grid needs a hash function, a hash function must surply the
    * following interface
    *
    *   size_t operator(int ,int , int)const
    *   size_t size()const
    *   void resize(size_t)
    *
    * Important: The hash function is responsible for picking the
    * proper hash table size. Thus if resize is invoked and the
    * implemented hash function for instance implements something
    * like:
    *
    *     m_size  = new_size + (new_size % 10)
    *
    * In which case the hash grid will automatically make sure that m_size is used
    * to allocate the number of hash cells. The size() method is expected to return
    * the value of m_size.
    *
    * The operator(int,int,int) should convert a discretized point into a 1D
    * hash key. The hash key value is expected to be within the
    * interval 0..m_size-1.
    *
    */
    template <
      typename real_vector3
      , typename int_vector3
      , typename data_type_
      , typename hash_function_type
    >
    class Grid
    {
    public:
      typedef          data_type_                                   data_type;
      typedef typename real_vector3::value_type                     real_type;
      typedef          int_vector3                                  triplet_type;   ///< Type of discretized points.
      typedef          real_vector3                                 point_type;     ///< Type of continuous points.

    protected:

      typedef std::vector<data_type*>                               data_ptr_container;
      typedef typename data_ptr_container::iterator                 data_ptr_iterator;
      typedef typename data_ptr_container::const_iterator           const_data_ptr_iterator;

    protected:

      class Cell
      {
      public:

        typedef boost::indirect_iterator<data_ptr_iterator,data_type> data_iterator;

      public:

        size_t        m_time_stamp;   ///< Timestamp, used to mark last query when data were stored in the cell.
        Grid          * m_owner;        ///< The hash table the cell belongs to.
        data_ptr_container  m_data;         ///< Data stored in hash cell
        size_t        m_query_stamp;  ///< Timestamp, used to guard against hash-collisions.
        size_t  m_next;               ///< Next free position in data ptr container.


      public:

        Cell()
          : m_time_stamp(0)
          , m_owner(0)
          , m_query_stamp(0)
        {
          m_next= 0;
          m_data.resize(16);
        }

        Cell( Grid * owner )
          : m_time_stamp(0)
          , m_owner(owner)
          , m_query_stamp(0)
        {
          m_next= 0;
          m_data.resize(16);
        }

        /*explicit*/ Cell( Cell const & cell ) { *this = cell; }

        Cell & operator=(Cell const & cell )
        {
          m_time_stamp = cell.m_time_stamp;
          m_owner = cell.m_owner;
          m_data = cell.m_data;
          m_query_stamp = cell.m_query_stamp;
          m_next = cell.m_next;
          return *this;
        }

      public:

        data_iterator begin()
        {
          assert(m_owner);
          if (m_time_stamp == m_owner->m_time_stamp)
            return data_iterator(m_data.begin());
          return data_iterator(m_data.end());
        }

        data_iterator end()
        {
          assert(m_owner);
          //return data_iterator(m_data.end());
          return data_iterator(m_data.begin()+m_next);
        }

      public:

        void add(data_type const & data)
        {
          assert(m_owner);
          if( m_owner->m_time_stamp != m_time_stamp )
          {
            //m_data.clear();
            m_next = 0;
            m_time_stamp = m_owner->m_time_stamp;
          }
          if(m_next==m_data.size())
          {
            data_ptr_container tmp;
            tmp = m_data;
            m_data.resize(2*m_next);
            std::copy(tmp.begin(),tmp.end(),m_data.begin());
          }
          m_data[m_next] = const_cast<data_type*>(&data);
          ++m_next;

          //m_data.push_back( &data );
        }

        bool remove(data_type & data)
        {
          data_type * tmp = const_cast<data_type*>(&data);
          assert(m_owner || !"Cell::remove(): owner was null");

          if( m_owner->m_time_stamp != m_time_stamp )//--- cell is (logically) empty!
            return false;

          if(m_next == 0)//--- cell do not contain any data
            return false;

          assert(m_next>0 || !"Cell::remove(): no data?");

          bool found = false;
          size_t target = 0;
          for(size_t i=0;i<m_next;++i)
          {
            if( m_data[i] == tmp )
            {
              found = true;
              target = i;
              break;
            }
          }
          if(found)
          {
            m_data[target] = m_data[m_next-1];
            m_data[m_next-1] = 0;
            --m_next;
            return true;
          }
          return false;
        }

        bool empty() const
        {
          assert(m_owner);
          return (m_time_stamp != m_owner->m_time_stamp);
        }

        size_t size() const
        {
          if(empty())
            return 0;
          //return m_data.size();
          return m_next;
        }
      };

    public:

      typedef Cell                              cell_type;
      typedef typename std::vector< cell_type >     cell_storage;

    protected:

      size_t            m_time_stamp;       ///< Query time-stamp.
      real_type               m_delta;            ///< Grid cell spacing.
      hash_function_type      m_hash_function;
      cell_storage            m_cells;            ///< Hash table cells.

    public:

      typedef typename cell_storage::iterator cell_iterator;

      cell_iterator begin(){return m_cells.begin();}
      cell_iterator end(){return m_cells.end();}

    public:

      Grid( )
        : m_time_stamp(0)
        , m_delta(5.0)
      {
        resize(1000);
      }

      Grid( size_t size )
        : m_time_stamp(0)
        , m_delta(5.0)
      {
        resize( size );
      }

    public:

      void resize( size_t size )
      {
        m_hash_function.resize(size);
        m_cells.resize( m_hash_function.size(), Cell(this) );
      }

      size_t size( ) const  { return m_hash_function.size(); }

      /**
      * Set Grid Cell Spacing.
      * Observe that a grid cell is not the same as a hash-cell, data inside a
      * grid cell is mapped to a hash-cell, this is not a one-to-one mapping. In
      * fact multiple grid cells can be mapped to the same hash-cell (this is
      * called a hash-collision).
      *
      * To reduce the chance of this, try increase the hash-table size, this
      * will decrease the chance of hash-collisions.
      *
      * For point type data, the grid spacing should usually be set to the
      * average size of the query data used.
      *
      * For volumetric data, the grid spacing should usually be set to the
      * average size of the volumetric data.
      *
      * @param delta   The new grid cell spacing size, must be a positive number.
      */
      void set_spacing( real_type delta )
      {
        assert( delta > 0 );
        m_delta = delta;
      }

      real_type get_spacing( ) const { return m_delta; }

      /**
      * Point Discretization.
      *
      * @param p   A 3D point in continious space
      *
      * @return    A discretized point identifying the grid
      *            cell (not the hash cell!) which the contineous
      *            point lies inside.
      */
      triplet_type get_triplet( point_type const & point )const
      {
        typedef typename triplet_type::value_type value_type;
        assert( m_delta > 0 );
        return triplet_type(
          static_cast<value_type>( std::floor( point(0) / m_delta ) ),
          static_cast<value_type>( std::floor( point(1) / m_delta ) ),
          static_cast<value_type>( std::floor( point(2) / m_delta ) )
          );
      }

      /**
      * Find the hash cell containing the grid cell enclosing the continuous point.
      */
      cell_type & get_cell(point_type const & p)
      {
        triplet_type triplet = get_triplet(p);
        size_t hash_key = m_hash_function( triplet(0), triplet(1), triplet(2) );
        return m_cells[ hash_key ];
      }

      /**
      * Find the hash cell containing the grid cell enclosing the discretized point.
      */
      cell_type & get_cell(triplet_type const & triplet)
      {
        size_t hash_key = m_hash_function( triplet(0), triplet(1), triplet(2) );
        return m_cells[ hash_key ];
      }

      void clear()
      {
        ++m_time_stamp; //--- Lazy deallocation
      }
    };

  } // namespace spatial_hashing

} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_SPATIAL_HASHING_GRID_H
#endif
