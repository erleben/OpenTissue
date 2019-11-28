#ifndef OPENTISSUE_COLLISION_SPATIAL_HASHING_SPATIAL_HASHING_QUERY_H
#define OPENTISSUE_COLLISION_SPATIAL_HASHING_SPATIAL_HASHING_QUERY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/cast.hpp> // needed for boost::numeric_cast

#include <list>
#include <map>
#include <cassert>

namespace OpenTissue
{
  namespace spatial_hashing
  {

    /**
    * Spatial Query Class.
    * This class is a generic type for all types of queries on a spatial
    * hash grid. All queries should be inherited from this class.
    *
    * Notice, that the report behavior are controlled by passing along
    * a tag to the queries. The tag indicates wether the query will
    * guard against reporting multiple identical (data, query) pairs.
    * Optional the end user can implement such guarding himself in
    * the collision_policy::report method.
    *
    * End user must supply a collision_policy type which defines two methods:
    *
    *   reset(results)
    *   report(data_type,query_type, results_container)
    *
    * and depeding on the type of query the collision policy also surplies
    * all or a subset of following methods
    *
    *          point_type position( data_type )       // only for point_data_query
    *          point_type min_coord data_type )       // only for aabb_data_query
    *          point_type max_coord( data_type )      // only for aabb_data_query
    *          point_type min_coord( query_type )     // for points, line and aabb data queries
    *          point_type max_coord( query_type )     // for points, line and aabb data queries
    *          point_type origin( data_type )         // only for line_data_query
    *          point_type destination( data_type )    // only for line_data_query
    *
    * The report method must implement a collision test, this could for
    * instance be a point in box test. The report method is responsible
    * for adding collision test results to the result container, also the
    * report method is responsible for guarding against self-collisions
    * and possible double reported  pairs (with order exchanged).
    *
    * The reset method is responsible for making the results container ready
    * for a new query. This may be an advantage if one wants to re-use allocated
    * memory for collision results.
    *
    *
    * The spatial query expects a hash_grid type as a template
    * argument. This hash_grid type must support the following
    * interface:
    *
    *    typedef ... cell_type
    *    typedef ... triplet_type
    *    typedef ... point_type
    *    typedef ... real_type
    *    typedef ... data_type
    *
    *    resize(new_size)
    *    set_spacing(new_spacing)
    *    cell_iterator  begin()
    *    cell_iterator  end()
    *    triplet_point get_triplet(point_type)
    *    cell_type & get_cell(triplet_type)
    *
    *  Hash cells must support the following interface:
    *
    *    m_query_stamp
    *    data_iterator begin()
    *    data_iterator end()
    *    empty()
    *    add( data_type )
    *
    */
    template<typename child_type,typename hash_grid, typename collision_policy>
    class Query 
      : public hash_grid
      , public collision_policy
    {
    protected:

      typedef typename hash_grid::cell_type        cell_type;
      typedef typename hash_grid::triplet_type     triplet_type;
      typedef typename hash_grid::point_type       point_type;
      typedef typename hash_grid::real_type        real_type;
      typedef typename hash_grid::data_type        data_type;
      typedef typename std::list<cell_type*>       cell_queue;
      typedef typename std::map<data_type*,bool>   data_tag_queue;

    protected:

      size_t m_query_stamp;  ///< Internally used time stamp, used to guard against hash collisions during queries.

    public:

      /**
      * No Collision Report Tag.
      * Unlike the all_tag, when this tag is used as report type, the
      * query will guard against collisions. That is, eventhough the
      * same data and query are found more than once during a query,
      * collision_policy::report is only invoked once.
      */
      struct no_collisions_tag {};

      /**
      * Report all Tag.
      * When this tag is used as report type all grid overlaps
      * are reported, even if they are redundant.
      * Example imagine a data_type is mapped into two differnt grid
      * cells. If a query type overlap both those grid cells, collision_policy::report is
      * invoked twice with the same data and query arguments.
      */
      struct all_tag {};

    public:

      /**
      * Full Query.
      * This is a two-pass query. In first pass data is mapped into a grid, in the
      * second pass query data is tested against content of overlapping grid cells.
      *
      * @param d0        Iterator to position of first data object.
      * @param d1        Iterator to position one past last data object.
      * @param q0        Iterator to position of first query object.
      * @param q1        Iterator to position one past last query object.
      * @param results   Upon return contains results of query.
      * @param type      This argument specifies the report type, possible types are all_tag or no_collisions_tag.
      */
      template< typename data_iterator, typename query_iterator,typename result_container,typename report_type >
      void operator()(
        data_iterator d0
        , data_iterator d1
        , query_iterator q0
        , query_iterator q1
        , result_container & results
        , report_type const & type
        )
      {
        child_type & self = static_cast<child_type &>(*this);
        this->clear();
        self.first_pass(d0,d1);

        collision_policy::reset(results);
        init_query();
        for(query_iterator q=q0;q!=q1;++q)
          query( (*q), results, type);
      }

      /**
      * Re-run query.
      * This method reruns the query on any previous mapped data.
      *
      * Prior to invoking this method either data should have been mapped into
      * grid by using the operator()(data_iter,data_iter) method, or a full
      * query should have been performed.
      *
      * @param q0        Iterator to position of first query object.
      * @param q1        Iterator to position one past last query object.
      * @param results   Upon return contains results of query.
      * @param type      This argument specifies the report type, possible types are all_tag or no_collisions_tag.
      */
      template< typename query_iterator,typename result_container,typename report_type >
      void operator()(
        query_iterator q0
        , query_iterator q1
        , result_container & results
        , report_type const & type
        )
      {
        collision_policy::reset(results);
        init_query();
        for(query_iterator q=q0;q!=q1;++q)
          query( (*q), results, type );
      }

      /**
      * Single Shoot Query.
      * This method runs a single shoot query on any previous mapped data.
      *
      * Prior to invoking this method either data should have been mapped into
      * grid by using the operator()(data_iter,data_iter) method, or a full
      * query should have been performed.
      *
      * @param q         Query data to perform query with.
      * @param results   Upon return contains results of query.
      * @param type      This argument specifies the report type, possible types are all_tag or no_collisions_tag.
      */
      template< typename query_type,typename result_container,typename report_type >
      void operator()( 
        query_type const & q
        , result_container & results
        , report_type const & type
        )
      {
        collision_policy::reset(results);
        query(q, results, type );
      }

      /**
      * Data Mapping
      * This method is provided in case the end-user wants greater control of
      * the two pass queries.
      *
      *  Example scenario:
      *
      *     query.init_data(d0,d1);
      *     for(each query_type q){
      *        query.run(q, results)
      *        std::cout << "near by neighbours of " << q << " are " << results << std::endl;
      *     }
      *
      *
      * @param d0        Iterator to position of first data object.
      * @param d1        Iterator to position one past last data object.
      */
      template< typename data_iterator  >
      void init_data(  data_iterator d0, data_iterator d1 )
      {
        child_type & self = static_cast<child_type &>(*this);
        this->clear();
        self.first_pass(d0,d1);
        init_query();
      }

      /**
      * Data Remapping.
      * This method maps the specified data without first clearing the spatial hashgrid.
      *
      *
      *
      */
      template< typename data_iterator  >
      void add_data(  data_iterator d0, data_iterator d1 )
      {
        child_type & self = static_cast<child_type &>(*this);
        self.first_pass(d0,d1);
      }

      template< typename data_iterator  >
      void remove_data(  data_iterator d0, data_iterator d1 )
      {
        child_type & self = static_cast<child_type &>(*this);
        self.remove_data(d0,d1);
      }

      /**
      * Automatically Initialization of Settings.
      * This method performs some simple statistics to find resonable
      * values for the hash table size and grid spacing.
      *
      * This method should only be invoked once during the life-time
      * of an application. If values are not satisfactory an end user
      * can always omit invoking this init method and set up hash table
      * size and grid spacing by invoking resize() method and
      * set_spacing() mehtod.
      *
      * @param begin    Iterator to position of first query object.
      * @param end      Iterator to position one past last query object.
      */
      template<typename iterator  >
      void auto_init_settings ( iterator begin, iterator end)
      {
        using std::max;
        using std::min;

        size_t cnt = 0;
        point_type mean;      
        mean.clear();

        for(iterator cur = begin;cur!=end;++cur,++cnt)
        {
          point_type d = collision_policy::max_coord(*cur) - collision_policy::min_coord(*cur);//--- min_coord and max_coord by collision_policy
          mean += d;
        }
        mean /= boost::numeric_cast<typename point_type::value_type>( cnt );
        size_t size = cnt;//--- KE 06-05-2005: Hmmm, cnt , appears to be a better value than  cnt/ 4?
        real_type spacing =  max (mean(0),  max( mean(1), mean(2) ) );
        this->resize( size );
        hash_grid::set_spacing( spacing  );
      }

    protected:

      void init_query()
      {
        m_query_stamp = 0;
        typename hash_grid::cell_iterator Cbegin = this->begin();
        typename hash_grid::cell_iterator Cend = this->end();
        for( typename hash_grid::cell_iterator cell = Cbegin;cell!=Cend;++cell)
          cell->m_query_stamp = m_query_stamp;
      }

      template< typename query_type,typename result_container >
      void query(query_type const & query, result_container & results, no_collisions_tag )
      {
        assert(m_query_stamp<~0u || !"Query stamp overflow, did you forget to map data?");

        ++m_query_stamp;

        point_type       min_corner = min_coord(query); //--- by collision_policy
        point_type       max_corner = max_coord(query); //--- by collision_policy
        triplet_type     m          = get_triplet(min_corner);
        triplet_type     M          = get_triplet(max_corner);

        assert( m(0) <= M(0) || !"Minimum was larger than maximum");
        assert( m(1) <= M(1) || !"Minimum was larger than maximum");
        assert( m(2) <= M(2) || !"Minimum was larger than maximum");

        triplet_type     triplet(m);
        data_tag_queue   data_tag;
        for ( triplet(0)= m(0) ; triplet(0) <= M(0); ++triplet(0) )
          for ( triplet(1)= m(1) ; triplet(1) <= M(1); ++triplet(1) )
            for ( triplet(2)= m(2) ; triplet(2) <= M(2); ++triplet(2) )
            {
              cell_type & cell = hash_grid::get_cell(triplet);

              if(cell.empty())
                continue;

              if(cell.m_query_stamp==m_query_stamp)
                continue;
              cell.m_query_stamp=m_query_stamp;

              typename cell_type::data_iterator Dbegin = cell.begin();
              typename cell_type::data_iterator Dend = cell.end();
              for( typename cell_type::data_iterator data = Dbegin;data!=Dend;++data)
              {
                bool & seen_before = data_tag[ &(*data) ];
                if(seen_before)
                  continue;
                seen_before = true;

                collision_policy::report( (*data), query, results);
              }
            }
      }

      template< typename query_type,typename result_container >
      void query(query_type const & query, result_container & results,  all_tag )
      {
        assert(m_query_stamp<~0u || !"Query stamp overflow, did you forget to map data?");

        ++m_query_stamp;

        point_type       min_corner = collision_policy::min_coord(query);  //--- by collision policy
        point_type       max_corner = collision_policy::max_coord(query);  //--- by collision policy
        triplet_type     m          = hash_grid::get_triplet(min_corner);
        triplet_type     M          = hash_grid::get_triplet(max_corner);

        assert( m(0) <= M(0) || !"Minimum was larger than maximum");
        assert( m(1) <= M(1) || !"Minimum was larger than maximum");
        assert( m(2) <= M(2) || !"Minimum was larger than maximum");

        triplet_type     triplet(m);
        for ( triplet(0)= m(0) ; triplet(0) <= M(0); ++triplet(0) )
          for ( triplet(1)= m(1) ; triplet(1) <= M(1); ++triplet(1) )
            for ( triplet(2)= m(2) ; triplet(2) <= M(2); ++triplet(2) )
            {
              cell_type & cell = hash_grid::get_cell(triplet);

              if(cell.empty())
                continue;

              if(cell.m_query_stamp==m_query_stamp)
                continue;
              cell.m_query_stamp=m_query_stamp;

              typename cell_type::data_iterator Dbegin = cell.begin();
              typename cell_type::data_iterator Dend = cell.end();
              for(typename cell_type::data_iterator data = Dbegin;data!=Dend;++data)
                collision_policy::report( (*data), query, results);
            }
      }

    };

  } // namespace spatial_hashing
} // namespace OpenTissue

// OPENTISSUE_COLLISION_SPATIAL_HASHING_SPATIAL_HASHING_QUERY_H
#endif
