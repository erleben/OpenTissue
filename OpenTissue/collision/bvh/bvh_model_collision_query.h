#ifndef OPENTISSUE_BVH_BVH_MODEL_COLLISION_QUERY_H
#define OPENTISSUE_BVH_BVH_MODEL_COLLISION_QUERY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp> // needed for boost::const_pointer_cast

namespace OpenTissue
{
  namespace bvh
  {
    /**
    * Model Frame Query.
    * This query assems that bvh A needs to be transformed into bvh B, this is
    * called a model update. Thus we call this a model collision query.
    */
    template <typename collision_policy>
    class ModelCollisionQuery : public collision_policy
    {
    public:

      /**
      * Model Frame Query.
      *
      * @param A2B       Model transform, brings bvh A into same frame as bvh B.
      * @param bvh_A     bvh A.
      * @param bvh_B     bvh B
      * @param results   Upon return this container contains any results from the
      *                  collision query.
      */
      template<typename coordsys_type,typename bvh_type, typename results_container>
      void run( coordsys_type const & A2B, bvh_type const & bvh_A, bvh_type const & bvh_B, results_container & results )
      {
        typedef typename bvh_type::bv_type                  bv_type;
        typedef typename bvh_type::bv_ptr                   bv_ptr;
        typedef typename bvh_type::bv_ptr_container         bv_ptr_container;
        typedef typename bvh_type::bv_ptr_iterator          bv_ptr_iterator;

        this->reset(results);//--- collision_policy

        bv_ptr_container Q;
        bv_ptr root_A = boost::const_pointer_cast<bv_type>( bvh_A.root() );
        bv_ptr root_B = boost::const_pointer_cast<bv_type>( bvh_B.root() );
        Q.push_back( root_A );
        Q.push_back( root_B );
        while ( !Q.empty() )
        {
          bv_ptr A( Q.front() );
          Q.pop_front();
          bv_ptr B( Q.front() );
          Q.pop_front();
          if( !this->overlap( A2B, A, B ) ) //--- collision_policy
            continue;
          if ( A->is_leaf() && B->is_leaf() )
          {
            this->report( A2B, A, B, results );  //--- collision_policy
            continue;
          }
          if (  B->is_leaf()  || ( !A->is_leaf() && (   A->volume().volume() > B->volume().volume()  )  ) )
          {
            bv_ptr_iterator a   = A->child_ptr_begin();
            bv_ptr_iterator end = A->child_ptr_end();
            for(;a!=end;++a)
            {
              bv_ptr ptr( *a );
              Q.push_back( ptr );
              Q.push_back( B );
            }
          }
          else
          {
            bv_ptr_iterator b   = B->child_ptr_begin();
            bv_ptr_iterator end = B->child_ptr_end();
            for(;b!=end;++b)
            {
              bv_ptr ptr( *b );
              Q.push_back( A );
              Q.push_back( ptr );
            }
          }
        }
      }
    };


  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_BVH_BVH_MODEL_COLLISION_QUERY_H
#endif
