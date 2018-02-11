#ifndef OPENTISSUE_BVH_BVH_SELF_COLLISION_QUERY_H
#define OPENTISSUE_BVH_BVH_SELF_COLLISION_QUERY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp> // needed for boost::const_pointer_cast

#include <list>

namespace OpenTissue
{
  namespace bvh
  {
    /**
    * Self Collision Query.
    *
    *  The collision policy must support the following interface:
    *
    *   bool adjacent(bv_ptr_type,bv_ptr_type)
    *   bool curvature_test(bv_ptr_type)
    *   bool curvature_test(bv_ptr_type,bv_ptr_type)
    *   bool overlap(bv_ptr_type,bv_ptr_type)
    *   bool report(bv_ptr_type,bv_ptr_type,result_type)
    *
    * In 2D the curvature test is a test for flatness of the surface, if
    * it is not bend enough for being self-intersecting the curvature test
    * is passed i.e. returns true.  If surface is severly bend the curvature
    * test should fail.
    *
    * For volumetric meshes, it is not easy to define what is meant by a
    * curvature test. However this implementation allows for a equivalent
    * test to be used.
    */
    template <typename collision_policy>
    class SelfCollisionQuery : public collision_policy
    {
    public:

      //--- Convenience stuff for better readability
      typedef typename collision_policy::bvh_type     bvh_type;
      typedef typename bvh_type::bv_ptr          bv_ptr;
      typedef typename bvh_type::bv_ptr_container     bv_ptr_container;
      typedef typename bvh_type::bv_type              bv_type;
      typedef typename bv_type::bv_ptr_iterator       bv_ptr_iterator;

    public:

      /**
      * Run Collision Query.
      *
      * @param bvh       The bvh upon which to perform self-collision.
      * @param results   Upon return this container contains any results from the
      *                  collision query.
      */
      template<typename results_container>
      void run( bvh_type const & bvh, results_container & results )
      {
        this->reset(results);//--- from collision_policy

        bv_ptr root = boost::const_pointer_cast<bv_type>( bvh.root() );

        self_test( root, results );
      }

    protected:

      /**
      * Self-test
      *
      * @param bv        A pointer to the bv that is current being tested agasint it-self.
      * @param results   Upon return this container contains any results from the
      *                  collision query.
      */
      template<typename results_container>
      void self_test( bv_ptr bv, results_container & results )
      {
        if( bv->is_leaf() )
          return;
        if( this->curvature_test( bv ) )  //--- collision_policy
          return;

        bv_ptr_iterator a   = bv->child_ptr_begin();
        bv_ptr_iterator end = bv->child_ptr_end();
        for(;a!=end;++a)
        {
          bv_ptr ptr1( *a );
          self_test( ptr1, results );
          bv_ptr_iterator b = a;
          ++b;
          for ( ;b != end; ++b )
          {
            bv_ptr ptr2( *b );
            tandem_test( ptr1, ptr2, results );
          }
        }
      }

      /**
      * Run Tandem Test.
      *
      * @param bv_A      Pointer to a bv.
      * @param bv_B      Pointer to another bv.
      * @param results   Upon return this container contains any results from the
      *                  collision query.
      */
      template<typename results_container>
      void tandem_test( bv_ptr bv_A, bv_ptr bv_B, results_container & results )
      {
        std::list<bool> adj_queue;
        adj_queue.push_back( bool(true) );
        bv_ptr_container Q;
        Q.push_back( bv_A );
        Q.push_back( bv_B );
        while ( !Q.empty() )
        {
          bv_ptr A( Q.front() );
          Q.pop_front();
          bv_ptr B( Q.front() );
          Q.pop_front();
          bool adj = adj_queue.front(); adj_queue.pop_front();

          if( !this->overlap( A, B ) )  //--- collision_policy
            continue;

          if ( adj )
          {
            adj = this->adjacent(A,B);       //--- collision_policy
            if( adj && this->curvature_test( A, B ) )  //--- collision_policy
              continue;
          }
          if ( A->is_leaf() && B->is_leaf() )
          {
            this->report( A, B, results);//--- collision_policy
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
              adj_queue.push_back( adj );
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
              adj_queue.push_back( adj );
            }
          }
        }
      }

    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_BVH_BVH_SELF_COLLISION_QUERY_H
#endif
