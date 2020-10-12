#ifndef CURVATURE_MESH_CONSTRUCTUION_POLICY_H
#define CURVATURE_MESH_CONSTRUCTUION_POLICY_H
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

/**
* Bottom-Up Construction Policy.
*
*
* Suggested improvements:
*
*  1) Extend graph nodes and edges with traits, which they get
*     from the construction policy. This would mean that members
*     such as tag and priority should be defined by traits.
*
*  2) Instead of storing all covered faces, store a pointer to a
*     single face, use tags on faces to assign unique number
*     identifying the surface regions.
*
*  3) When mergin two regions, do a breadth first traversal over
*     all faces in the old regions and assing new surface region
*     number.  While doing so, edges on the boundary of the surface
*     region can be identified, and their end vertices can be added
*     to the adjacency list. This eliminates the need for searching
*     and identifying vertices inside the surface region.
*
*/
template<typename bvh_type_>
class CurvatureMeshConstructionPolicy 
  : public OpenTissue::bvh::BinaryMatchBottomUpPolicy<bvh_type_>
{
public:

  //--- Conenience stuff for better readability
  typedef          bvh_type_                          bvh_type;
  typedef OpenTissue::bvh::BVHGraph<bvh_type>              graph_type;
  typedef typename graph_type::node_ptr_type          node_ptr_type;
  typedef typename graph_type::edge_ptr_type          edge_ptr_type;
  typedef typename graph_type::edge_iterator          edge_iterator;
  typedef typename graph_type::node_iterator          node_iterator;
  typedef typename graph_type::real_type              real_type;
  typedef typename bvh_type::bv_type                  bv_type;
  typedef typename bvh_type::bv_ptr                   bv_ptr;
  typedef typename bvh_type::bv_iterator              bv_iterator;
  typedef typename bvh_type::annotated_bv_ptr         annotated_bv_ptr;
  typedef typename bvh_type::annotated_bv_type        annotated_bv_type;
  typedef typename bvh_type::volume_type              aabb_type;
  typedef typename bvh_type::geometry_type            geometry_type;
  typedef typename bvh_type::geometry_iterator        geometry_iterator;
  typedef typename aabb_type::math_types              math_types;
  typedef typename math_types::vector3_type           vector3_type;
  typedef typename math_types::value_traits           value_traits;
  typedef typename OpenTissue::polymesh::PolyMesh<>   mesh_type;
  typedef typename mesh_type::face_type               face_type;
  typedef typename mesh_type::vertex_type             vertex_type;
  typedef typename mesh_type::halfedge_type           halfedge_type;
  typedef typename bv_type::adjacency_iterator        adjacency_iterator;

public:

  CurvatureMeshConstructionPolicy(){}

  virtual ~CurvatureMeshConstructionPolicy(){}

protected:

  const unsigned int degree( void ) const {  return 8; }

public:

  void init(graph_type & graph)
  {
    OpenTissue::bvh::BinaryMatchBottomUpPolicy<bvh_type>::init(graph);
    for(node_iterator node = graph.node_begin();node!=graph.node_end();++node)
    {
      assert(node->bv()->is_leaf());

      annotated_bv_ptr bv = boost::static_pointer_cast<annotated_bv_type>(node->bv());

      face_type * geometry = *(bv->geometry_begin());

      mesh_type::face_vertex_circulator v(*geometry),vend;
      for(;v!=vend;++v)
      {
        bv->m_adjacency.push_back(  &(*v) );
      }
      geometry->m_tag = 0;
      assert(node->bv()->m_adjacency.size()>0);
    }
  }

  template<typename geometry_iterator,typename volume_iterator>
    aabb_type fit(geometry_iterator g0,geometry_iterator g1,volume_iterator v0,volume_iterator v1)
  {
    unsigned int  vN = std::distance(v0,v1);
    if(vN>1)
      return fit_volumes(v0,v1);
    return fit_geometry(g0,g1);
  }

  void update(node_ptr_type node)
  {
    OpenTissue::bvh::BinaryMatchBottomUpPolicy<bvh_type>::update(node);
    if(node->bv())
    {
      //---
      //--- Add the union of all child adjacency information to the new bv
      //---
      bv_ptr bv = node->bv();
      for(bv_iterator child = bv->child_begin();child!=bv->child_end();++child)
      {
        for(adjacency_iterator adj=child->m_adjacency.begin();adj!=child->m_adjacency.end();++adj)
        {
          adjacency_iterator lookup = find(bv->m_adjacency.begin(),bv->m_adjacency.end(), (*adj) );
          if(lookup==bv->m_adjacency.end())
            bv->m_adjacency.push_back( (*adj) );
          (*adj)->m_tag = 0;
        }
      }
      //---
      //--- for each vertex count the number of neighboring faces lying in the surface region
      //---
      for(geometry_iterator face = node->coverage().begin();face!=node->coverage().end();++face)
      {
        (*face)->m_tag = 1;
        mesh_type::face_vertex_circulator v( *(*face) ),vend;
        for( ; v!=vend; ++v )
          ++v->m_tag;
      }
      //--- Remove vertices where the number of total neighboring faces is equal to
      //--- the number of neighboring faces lying in the surface region
      //---
      //---   These vertices must lie inside the surface region, thus they are not
      //---   contributing to the boundary information.
      //---
      for(adjacency_iterator adj=bv->m_adjacency.begin();adj!=bv->m_adjacency.end();)
      {
        adjacency_iterator target = adj;
        ++adj;
        (*target)->m_tag = valency(*(*target)) - (*target)->m_tag;
        assert((*target)->m_tag>=0);
        if((*target)->m_tag==0)
        {
          //--- this means that verex only had neighboring faces lying in the surface region
          bv->m_adjacency.erase(target);
        }
      }
      //--- Boundary may still contain redudant information, clearly if a vertex
      //--- have more than 2 neighboring faces lying outside the surface region, it
      //--- can not be removed without causing loss of boundary information.
      //---
      //--- Vertices with exactly one neighboring face lying out side the surface
      //--- region should not exist if we are working on a triangle mesh, however
      //--- for the more general case, they might appear. They can however be safely
      //--- removed, since there must be other vertices on the boundary contributing
      //--- with the same boundary information.
      //---
      //--- Vertices with exactly two neighboring faces outside the surface region, may
      //--- be removed if the remaing vertices in the adjacency list indicates that
      //--- the two faces are neighbors.
      for(adjacency_iterator adj=bv->m_adjacency.begin();adj!=bv->m_adjacency.end();)
      {
        adjacency_iterator target = adj;
        ++adj;
        if((*target)->m_tag==1)
          bv->m_adjacency.erase(target);
        else if((*target)->m_tag==2)
        {
          //--- sorry not implemented!!!
        }
      }
      //--- reset face tages in coverage
      for(geometry_iterator face = node->coverage().begin();face!=node->coverage().end();++face)
        (*face)->m_tag = 0;
    }
  }

private:

  template<typename geometry_iterator>
    aabb_type fit_geometry( geometry_iterator begin,geometry_iterator end )
  {
    vector3_type pmin,pmax;
    pmin = vector3_type( value_traits::infinity(), value_traits::infinity(), value_traits::infinity());
    pmax = -pmin;
    for ( geometry_iterator geometry = begin;geometry!=end;++geometry)
    {
      mesh_type::face_vertex_circulator v( *(*geometry)),vend;
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
    return aabb_type(pmin,pmax);
  }
  
  template<typename volume_iterator>
    aabb_type fit_volumes( volume_iterator begin, volume_iterator end )
  {
    vector3_type pmin,pmax;
    pmin = vector3_type( value_traits::infinity(), value_traits::infinity(), value_traits::infinity());
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

};


// CURVATURE_MESH_CONSTRUCTUION_POLICY_H
#endif
