#ifndef OPENTISSUE_COLLISION_BVH_BVH_BOTTOM_UP_REFITTER_H
#define OPENTISSUE_COLLISION_BVH_BVH_BOTTOM_UP_REFITTER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp>

namespace OpenTissue
{

  namespace bvh
  {

    /**
    * Classical Bottom-Up refitting.
    */
    template <typename volume_refitter>
    class BottomUpRefitter 
      : public volume_refitter
    {
    public:

      typedef typename volume_refitter::bvh_type      bvh_type;
      typedef typename bvh_type::bv_ptr               bv_ptr;
      typedef typename bvh_type::bv_type              bv_type;
      typedef typename bvh_type::bv_ptr_container     bv_ptr_container;

    public:

      /**
      * Run update algorithm.
      *
      * @param bvh    Reference to BVH that should be updated.
      */
      void run( bv_ptr_container const & leaves )
      {
        bv_ptr_container Q;

        //Q.insert(Q.end(),leaves.begin(),leaves.end());
        std::copy( leaves.begin(), leaves.end(), std::back_inserter( Q ) );

        while ( !Q.empty() )
        {
          bv_ptr bv( Q.front() );
          Q.pop_front();
          this->refit(bv);   //--- From volume refitter policy!!!
          if ( bv->parent() )
          {
            bv_ptr ptr = boost::const_pointer_cast<bv_type>( bv->parent() ) ;
            Q.push_back( ptr );
          }
        }
      }

    };

  } // namespace bvh

} // namespace OpenTissue

// OPENTISSUE_COLLISION_BVH_BVH_BOTTOM_UP_REFITTER_H
#endif
