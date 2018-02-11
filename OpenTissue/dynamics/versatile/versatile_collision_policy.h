#ifndef OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_COLLISION_POLICY_H
#define OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_COLLISION_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/spatial_hashing/spatial_hashing.h>

namespace OpenTissue
{
  namespace versatile
  {
    namespace detail
    {

      template<typename versatile_types>
      class collision_policy
      {
      public:
        typedef typename versatile_types::node_type                    data_type;
        typedef typename versatile_types::tetrahedron_type             query_type;
        typedef typename versatile_types::value_traits                 value_traits;
        typedef typename versatile_types::real_type                    real_type;
        typedef typename versatile_types::vector3_type                 vector3_type;
        typedef OpenTissue::spatial_hashing::GridHashFunction          hash_function;
        //typedef OpenTissue::spatial_hashing::PrimeNumberHashFunction                 hash_function;
        //typedef OpenTissue::spatial_hashing::ShiftedGoldenMeanHashFunction           hash_function;
        //typedef OpenTissue::spatial_hashing::RandomArrayHashFunction                 hash_function;

        typedef OpenTissue::spatial_hashing::Grid< vector3_type, math::Vector3<int>, data_type, hash_function>  hash_grid;

      protected:

        class ResultType
        {
        public:
          data_type * m_data;       ///< Pointer to penetrating node.
          query_type * m_query;     ///< Pointer to penetrated tetrahedron.
          real_type  m_w0;          ///< Barycentric coordinates of the node inside the tetrahedron.
          real_type  m_w1;
          real_type  m_w2;
          real_type  m_w3;
        };

      public:

        typedef ResultType              result_type;
        typedef std::list<result_type>  result_container;

      public:

        vector3_type position(data_type const & data)const {return data.position();}
        vector3_type min_coord(query_type const & query)const {return query.min();}
        vector3_type max_coord(query_type const & query)const {return query.max();}

        void reset(result_container & /*results*/)  {}

        void report(data_type const & data, query_type const & query,result_container & results)
        {
          //--- Ignore the case where a node is part of the tetrahedron it is being tested against.
          if( data.owner()==query.owner()
            &&
            ( data.idx()==query.i()->idx() || data.idx()==query.j()->idx() || data.idx()==query.k()->idx() || data.idx()==query.m()->idx() )
            )
            return;

          vector3_type const & pi = query.i()->m_coord;
          vector3_type const & pj = query.j()->m_coord;
          vector3_type const & pk = query.k()->m_coord;
          vector3_type const & pm = query.m()->m_coord;
          vector3_type const & p  = data.m_coord;

          real_type const delta = boost::numeric_cast<real_type>( 10e-5 );

          real_type lower = - delta;
          real_type upper = value_traits::one() + delta;
          result_type result;

          
          OpenTissue::geometry::barycentric_algebraic(pi,pj,pk,pm,p,result.m_w0,result.m_w1,result.m_w2,result.m_w3);
          if(
            (result.m_w0>lower)&&(result.m_w0<upper)
            &&
            (result.m_w1>lower)&&(result.m_w1<upper)
            &&
            (result.m_w2>lower)&&(result.m_w2<upper)
            &&
            (result.m_w3>lower)&&(result.m_w3<upper)
            )
          {
            result.m_data = const_cast<data_type*>( &data );
            result.m_query = const_cast<query_type*>( &query );
            results.push_back( result );
            return;
          }
        }
      };

    } // namespace detail
  } // namespace versatile
} // namespace OpenTissue

//OPENTISSUE_DYNAMICS_VERSATILE_VERSATILE_COLLISION_POLICY_H
#endif
