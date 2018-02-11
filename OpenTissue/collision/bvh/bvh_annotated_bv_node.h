#ifndef OPENTISSUE_COLLISION_BVH_BVH_ANNOTATED_BV_NODE_H
#define OPENTISSUE_COLLISION_BVH_BVH_ANNOTATED_BV_NODE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh_bv_node.h>

namespace OpenTissue
{
  namespace bvh
  {

    /**
    * Annotated BV Class.
    * A annotated BV, which is capable of storing geometry.
    */
    template <typename B, typename T>
    class AnnotatedBV 
      : public BV<B, T>
    {
    public:

      typedef  B                                          bvh_type;
      typedef typename bvh_type::geometry_type            geometry_type;
      typedef typename bvh_type::geometry_container       geometry_container;
      typedef typename bvh_type::geometry_iterator        geometry_iterator;
      typedef typename bvh_type::geometry_const_iterator  geometry_const_iterator;

    protected:

      geometry_container m_geometry;    ///< Geometry attached to this BV.

    public:

      AnnotatedBV()
        : BV<B,T>()
        , m_geometry()
      {
        this->m_has_geometry=true;
      }

    public:

      geometry_iterator       geometry_begin()       { return m_geometry.begin(); }
      geometry_iterator       geometry_end()         { return m_geometry.end();   }
      geometry_const_iterator geometry_begin() const { return m_geometry.begin(); }
      geometry_const_iterator geometry_end()   const { return m_geometry.end();   }

      void insert( geometry_type const & G ) { m_geometry.push_back( G ); }

      void insert( geometry_container const & G ) { std::copy( G.begin(), G.end(), std::back_inserter( m_geometry ) ); }
    };

  } // namespace bvh

} // namespace OpenTissue

//OPENTISSUE_COLLISION_BVH_BVH_ANNOTATED_BV_NODE_H
#endif
