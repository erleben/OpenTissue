#ifndef T4MESH_VOLUME_REFITTER_H
#define T4MESH_VOLUME_REFITTER_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <boost/shared_ptr.hpp>

#include <vector>

template<typename bvh_type_>
class T4MeshVolumeRefitter
{
public:

  typedef          bvh_type_                                       bvh_type;
  typedef typename bvh_type::volume_type                           volume_type;
  typedef typename bvh_type::geometry_type                         geometry_type;
  typedef typename bvh_type::bv_ptr                                bv_ptr;
  typedef typename bvh_type::annotated_bv_ptr                      annotated_bv_ptr;
  typedef typename bvh_type::annotated_bv_type                     annotated_bv_type;
  typedef typename bvh_type::bv_iterator                           bv_iterator;
  typedef typename bvh_type::geometry_iterator                     geometry_iterator;

  typedef typename volume_type::math_types                         math_types;

  typedef typename math_types::vector3_type                        vector3_type;
  typedef typename math_types::value_traits                        value_traits;
  typedef std::vector<vector3_type>                                coord_container;

protected:

  coord_container * m_coords;   ///< Pointer to coordiates of mesh. Must be set by invoking set_coords(...) method.

public:

  void set_coords(coord_container & coords)
  {
    m_coords = &coords;
  }

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

      unsigned int i = geometry->i()->idx();
      unsigned int j = geometry->j()->idx();
      unsigned int k = geometry->k()->idx();
      unsigned int m = geometry->m()->idx();
      if((*m_coords)[i](0) < pmin(0))
        pmin(0) = (*m_coords)[i](0);
      if((*m_coords)[i](1) < pmin(1))
        pmin(1) = (*m_coords)[i](1);
      if((*m_coords)[i](2) < pmin(2))
        pmin(2) = (*m_coords)[i](2);
      if((*m_coords)[i](0) > pmax(0))
        pmax(0) = (*m_coords)[i](0);
      if((*m_coords)[i](1) > pmax(1))
        pmax(1) = (*m_coords)[i](1);
      if((*m_coords)[i](2) > pmax(2))
        pmax(2) = (*m_coords)[i](2);
      if((*m_coords)[j](0) < pmin(0))
        pmin(0) = (*m_coords)[j](0);
      if((*m_coords)[j](1) < pmin(1))
        pmin(1) = (*m_coords)[j](1);
      if((*m_coords)[j](2) < pmin(2))
        pmin(2) = (*m_coords)[j](2);
      if((*m_coords)[j](0) > pmax(0))
        pmax(0) = (*m_coords)[j](0);
      if((*m_coords)[j](1) > pmax(1))
        pmax(1) = (*m_coords)[j](1);
      if((*m_coords)[j](2) > pmax(2))
        pmax(2) = (*m_coords)[j](2);
      if((*m_coords)[m](0) < pmin(0))
        pmin(0) = (*m_coords)[m](0);
      if((*m_coords)[m](1) < pmin(1))
        pmin(1) = (*m_coords)[m](1);
      if((*m_coords)[m](2) < pmin(2))
        pmin(2) = (*m_coords)[m](2);
      if((*m_coords)[m](0) > pmax(0))
        pmax(0) = (*m_coords)[m](0);
      if((*m_coords)[m](1) > pmax(1))
        pmax(1) = (*m_coords)[m](1);
      if((*m_coords)[m](2) > pmax(2))
        pmax(2) = (*m_coords)[m](2);
      if((*m_coords)[k](0) < pmin(0))
        pmin(0) = (*m_coords)[k](0);
      if((*m_coords)[k](1) < pmin(1))
        pmin(1) = (*m_coords)[k](1);
      if((*m_coords)[k](2) < pmin(2))
        pmin(2) = (*m_coords)[k](2);
      if((*m_coords)[k](0) > pmax(0))
        pmax(0) = (*m_coords)[k](0);
      if((*m_coords)[k](1) > pmax(1))
        pmax(1) = (*m_coords)[k](1);
      if((*m_coords)[k](2) > pmax(2))
        pmax(2) = (*m_coords)[k](2);
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

// T4MESH_VOLUME_REFITTER_H
#endif
