#ifndef VOLUME_REFITTER_H
#define VOLUME_REFITTER_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp>

template<typename bvh_type_,typename mesh_type_>
class VolumeRefitter
{
public:

  typedef          bvh_type_                          bvh_type;
  typedef          mesh_type_                         mesh_type;

  typedef typename bvh_type::volume_type              volume_type;
  typedef typename volume_type::math_types            math_types;
  typedef typename math_types::vector3_type           vector3_type;
  typedef typename math_types::value_traits           value_traits;
  typedef typename bvh_type::geometry_type            geometry_type;

  typedef typename bvh_type::bv_ptr                   bv_ptr;
  typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;
  typedef typename bvh_type::annotated_bv_type        annotated_bv_type;
  typedef typename bvh_type::bv_iterator              bv_iterator;
  typedef typename bvh_type::geometry_iterator        geometry_iterator;

public:

  /**
  * Refit volume of BV node.
  *
  * @param bv    A pointer to the BV node that should be refitted.
  */
  void refit(bv_ptr bv)
  {
    assert(bv);
    vector3_type & pmin = bv->volume().min();
    vector3_type & pmax = bv->volume().max();
    pmin = vector3_type(value_traits::infinity());
    pmax = -pmin;
    if(bv->is_leaf())
    {
      annotated_bv_ptr annotated_bv = boost::static_pointer_cast<annotated_bv_type>(bv);

      geometry_type geometry = *(annotated_bv->geometry_begin());

      typename mesh_type::face_vertex_circulator v(*geometry),vend;
      for(;v!=vend;++v)
      {
        vector3_type & coord = v->m_coord;
        if(coord(0) < pmin(0))
          pmin(0) = coord(0);
        if(coord(1) < pmin(1))
          pmin(1) = coord(1);
        if(coord(2) < pmin(2))
          pmin(2) = coord(2);
        if(coord(0) > pmax(0))
          pmax(0) = coord(0);
        if(coord(1) > pmax(1))
          pmax(1) = coord(1);
        if(coord(2) > pmax(2))
          pmax(2) = coord(2);
      }
    }
    else
    {
      for ( bv_iterator child = bv->child_begin();child!=bv->child_end();++child )
      {
        vector3_type & min_corner = child->volume().min();
        vector3_type & max_corner = child->volume().max();
        if(min_corner(0) < pmin(0))
          pmin(0) = min_corner(0);
        if(min_corner(1) < pmin(1))
          pmin(1) = min_corner(1);
        if(min_corner(2) < pmin(2))
          pmin(2) = min_corner(2);
        if(max_corner(0) > pmax(0))
          pmax(0) = max_corner(0);
        if(max_corner(1) > pmax(1))
          pmax(1) = max_corner(1);
        if(max_corner(2) > pmax(2))
          pmax(2) = max_corner(2);
      }
    }
  }

};

// VOLUME_REFITTER_H
#endif 
