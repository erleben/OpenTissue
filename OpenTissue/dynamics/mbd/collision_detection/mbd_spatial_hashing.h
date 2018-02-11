#ifndef OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SPATIAL_HASHING_H
#define OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SPATIAL_HASHING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/collision/intersect/intersect_aabb_aabb.h>
#include <OpenTissue/collision/spatial_hashing/spatial_hashing.h>

namespace OpenTissue
{
  namespace mbd
  {
    /**
    * The Spatial Hashing Broad Phase Collision Detection Algorithm.
    */
    template <typename types>
    class SpatialHashing
    {
    protected:

      typedef typename types::math_policy::index_type      size_type;
      typedef typename types::math_policy::real_type       real_type;
      typedef typename types::math_policy::vector3_type    vector3_type;
      typedef typename types::math_policy::matrix3x3_type  matrix3x3_type;
      typedef typename types::configuration_type           configuration_type;
      typedef typename types::body_type                    body_type;
      typedef typename types::edge_type                    edge_type;
      typedef typename types::edge_ptr_container           edge_ptr_container;

    protected:

      class collision_policy
      {
      public:

        typedef typename types::math_policy::index_type  size_type;
        typedef typename types::math_policy::real_type   real_type;
        typedef body_type                   data_type;
        typedef body_type                   query_type;
        typedef edge_type                   result_type;
        typedef edge_ptr_container          result_container;

        typedef OpenTissue::spatial_hashing::PrimeNumberHashFunction                 hash_function;
        //typedef OpenTissue::spatial_hashing::GridHashFunction                        hash_function;
        //typedef OpenTissue::spatial_hashing::ShiftedGoldenMeanHashFunction           hash_function;
        //typedef OpenTissue::spatial_hashing::RandomArrayHashFunction                 hash_function;
        typedef OpenTissue::spatial_hashing::Grid< OpenTissue::math::Vector3<real_type>, OpenTissue::math::Vector3<int>, data_type, hash_function>  hash_grid;

      public:

        size_type m_policy_time_stamp;
        configuration_type * m_configuration;

      public:

        collision_policy()
          : m_policy_time_stamp(0)
        {}

        vector3_type min_coord(body_type const & body) const {   return body.min();};
        vector3_type max_coord(body_type const & body) const {   return body.max();};

        void reset(result_container & results)
        {
          results.clear();
          ++m_policy_time_stamp;
        };

        void report(data_type const & data, query_type const & query,result_container & results)
        {
          body_type * body1 = const_cast<body_type*>(&data);
          body_type * body2 = const_cast<body_type*>(&query);

          if(body1==body2)
            return;

          if(body1->get_index() > body2->get_index())
            return;

          edge_type * edge = m_configuration->get_edge( body1, body2 );
          if ( edge && edge->m_sh_time_stamp == m_policy_time_stamp )
            return;

          if(OpenTissue::intersect::aabb_aabb(body1->m_aabb,body2->m_aabb))
          {
            if ( !edge )
              edge = m_configuration->add( body1, body2 );
            edge->m_sh_time_stamp = m_policy_time_stamp;
            results.push_back( edge );
          }
        }
      };

      typedef OpenTissue::spatial_hashing::AABBDataQuery< typename collision_policy::hash_grid, collision_policy >  query_algorithm;

    public:

      class node_traits
      {
      public:

        geometry::AABB<typename types::math_policy> m_aabb;

        vector3_type min(void) const {  return m_aabb.min();};
        vector3_type max(void) const {  return m_aabb.max();};

        void updateAABB( body_type * self, real_type const & envelope )
        {
          vector3_type r;
          self->get_position( r );
          matrix3x3_type R;
          self->get_orientation( R );
          self->compute_collision_aabb( r, R, m_aabb.m_min, m_aabb.m_max, envelope );
        }
      };

      class edge_traits
      {
      public:
        edge_traits ( void ) : m_sh_time_stamp( 0 ) {};
        size_type m_sh_time_stamp;
      };

      class constraint_traits  { };

    protected:

      query_algorithm  m_query;

    public:

      SpatialHashing( ){}

    public:

      void clear()
      {
        this->m_query.m_policy_time_stamp = 0;
        this->m_query.m_configuration = 0;
      }

      void add( body_type * /*body*/ )    { };
      void remove( body_type * /*body*/ )    {};

      void init( configuration_type & configuration )
      {
        clear();
        m_query.m_policy_time_stamp = 0;
        m_query.m_configuration = &configuration;

        real_type envelope =  m_query.m_configuration->get_collision_envelope();

        typename configuration_type::body_iterator begin(  m_query.m_configuration->body_begin() );
        typename configuration_type::body_iterator end(  m_query.m_configuration->body_end() );
        typename configuration_type::body_iterator body;

        for ( body = begin; body != end; ++body )
          body->updateAABB( &( *body ), envelope );

        m_query.auto_init_settings(begin,end);

        std::cout << "SpatialHashing::init():  |dx| = " << m_query.get_spacing() << std::endl;
        std::cout << "SpatialHashing::init():   |S| = " << m_query.size() << std::endl;
      }

      void run( edge_ptr_container & edges )
      {
        real_type envelope = m_query.m_configuration->get_collision_envelope();

        typename configuration_type::body_iterator begin( m_query.m_configuration->body_begin() );
        typename configuration_type::body_iterator end( m_query.m_configuration->body_end() );
        typename configuration_type::body_iterator body;

        for ( body = begin; body!=end; ++body )
          body->updateAABB( &( *body ), envelope );

        //m_query(begin, end, begin, end, edges, query_algorithm::no_collisions_tag() );
        m_query(begin, end, begin, end, edges, typename query_algorithm::all_tag() );


        //{
        //  typename configuration_type::indirect_joint_iterator begin( m_query.m_configuration->joint_begin() );
        //  typename configuration_type::indirect_joint_iterator end( m_query.m_configuration->joint_end() );
        //  typename configuration_type::indirect_joint_iterator joint;
        //  for ( joint = begin; joint!=end; ++joint )
        //  {

        //    body_type * body1 = joint->get_socket_A()->get_body();
        //    body_type * body2 = joint->get_socket_B()->get_body();

        //    edge_type * edge = m_query.m_configuration->get_edge( body1, body2 );
        //    //if ( edge && edge->m_sh_time_stamp == m_query.m_policy_time_stamp )
        //    //  continue;
        //    if ( !edge )
        //      edge = m_query.m_configuration->add( body1, body2 );
        //    //edge->m_sh_time_stamp = m_query.m_policy_time_stamp;
        //    //edges.push_back( edge );
        //  }
        //}

      };

    };

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_COLLISION_DETECTION_MBD_SPATIAL_HASHING_H
#endif
