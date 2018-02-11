#ifndef T4MESH_CONSTRUCTION_POLICY_H
#define T4MESH_CONSTRUCTION_POLICY_H
//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2007 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/collision/bvh/bvh.h>

#include <boost/shared_ptr.hpp>

#include <vector>

template<typename bvh_type_>
class T4MeshConstructionPolicy 
  : public OpenTissue::bvh::DefaultPriorityBottomUpPolicy<bvh_type_>
{
public:

  typedef          bvh_type_                                bvh_type;
  typedef OpenTissue::bvh::BVHGraph<bvh_type>                    graph_type;
  typedef typename graph_type::node_ptr_type                node_ptr_type;
  typedef typename graph_type::edge_ptr_type                edge_ptr_type;
  typedef typename graph_type::edge_iterator                edge_iterator;
  typedef typename graph_type::real_type                    real_type;
  typedef typename bvh_type::bv_type                        bv_type;
  typedef typename bvh_type::volume_type                    aabb_type;
  typedef typename bvh_type::geometry_type                  geometry_type;
  typedef typename aabb_type::math_types                    math_types;
  typedef typename math_types::vector3_type                 vector3_type;
  typedef typename math_types::value_traits                 value_traits;
  typedef std::vector<vector3_type>                         coord_container;

protected:

  coord_container * m_coords;   ///< Pointer to coordiates of mesh. Must be set by invocing set_coords(...) method.

public:

  void set_coords(coord_container & coords) 
  { 
    m_coords = &coords; 
  }

  template<typename geometry_iterator>
  aabb_type fit_geometry( geometry_iterator begin,geometry_iterator end )
  {
    vector3_type pmin,pmax;
    pmin = vector3_type(value_traits::infinity());
    pmax = -pmin;
    for ( geometry_iterator geometry = begin; geometry != end; ++geometry )
    {
      unsigned int i = (*geometry)->i()->idx();
      unsigned int j = (*geometry)->j()->idx();
      unsigned int k = (*geometry)->k()->idx();
      unsigned int m = (*geometry)->m()->idx();
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
    return aabb_type(pmin,pmax);
  }

  template<typename volume_iterator>
  aabb_type fit_volumes( volume_iterator begin, volume_iterator end )
  {
    vector3_type pmin,pmax;
    pmin = vector3_type(value_traits::infinity());
    pmax = -pmin;
    for ( volume_iterator volume = begin;volume!=end;++volume )
    {
      vector3_type & min_corner = volume->min();
      vector3_type & max_corner = volume->max();
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
    return aabb_type(pmin,pmax);
  }

protected:

  const unsigned int degree( void ) const {  return 8; }

public:

  template<typename geometry_iterator,typename volume_iterator>
  aabb_type fit(geometry_iterator g0, geometry_iterator g1, volume_iterator v0, volume_iterator v1)
  {
    unsigned int  vN = std::distance(v0,v1);
    if(vN>1)
      return fit_volumes(v0,v1);
    return fit_geometry(g0,g1);
  }

};

// T4MESH_CONSTRUCTION_POLICY_H
#endif
