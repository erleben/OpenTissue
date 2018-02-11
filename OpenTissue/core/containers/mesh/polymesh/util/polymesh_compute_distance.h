#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DISTANCE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DISTANCE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_edge.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh_face.h>

#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_vertex_edge_voronoi_plane.h>
#include <OpenTissue/core/containers/mesh/polymesh/util/polymesh_compute_edge_face_voronoi_plane.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_compute_face_plane.h>


#include <OpenTissue/core/geometry/geometry_plane.h>                      //--- needed for plane type

#include <cmath>  //--- needed for sqrt

namespace OpenTissue
{
  namespace polymesh
  {

    template<typename mesh_type,typename vector3_type>
    typename vector3_type::value_type
      compute_distance(PolyMeshVertex<mesh_type> const & v,vector3_type const & p)
    {
      vector3_type tmp = p - v.m_coord;
      return std::sqrt(tmp*tmp);
    }

    template<typename mesh_type,typename vector3_type>
    typename vector3_type::value_type
      compute_distance(PolyMeshFace<mesh_type> const & f, vector3_type const & p)
    {
      typedef typename mesh_type::vertex_iterator            vertex_iterator;
      typedef typename mesh_type::face_halfedge_circulator   face_halfedge_circulator;
      typedef typename mesh_type::halfedge_iterator          halfedge_iterator;
      typedef typename mesh_type::face_iterator              face_iterator;

      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::real_type                 real_type;
      typedef geometry::Plane<math_types>                    plane_type;

      plane_type plane;

      face_halfedge_circulator h(f),end;
      for(;h!=end;++h)
      {
        vertex_iterator v = h->get_destination_iterator();
        compute_vertex_edge_voronoi_plane(*v,*h,plane);
        if(plane.signed_distance(p) >= 0)
          continue;

        //--- Note it is implicitly assumed that face is convex.
        halfedge_iterator last = h->get_owner()->halfedge_end();//  = 0;
        halfedge_iterator search  = h->get_owner()->get_halfedge_iterator(h->get_handle());
        bool forever = true; // NOTE: shut up the compiler. Is there a better way?
        do
        {
          compute_vertex_edge_voronoi_plane(
            *(search->get_destination_iterator())
            , *(search)
            , plane);

          if(plane.signed_distance(p)>0)
          {
            if(last==search->get_next_iterator())
            {
              return compute_distance(*(search->get_destination_iterator()),p);
            }
            last = search;
            search = search->get_next_iterator();
            continue;
          }

          compute_vertex_edge_voronoi_plane(
            *(search->get_origin_iterator())
            , *(search->get_twin_iterator())
            , plane);

          if(plane.signed_distance(p)>0)
          {
            if(last==search->get_prev_iterator())
            {
              return compute_distance(*(search->get_origin_iterator()),p);
            }
            last = search;
            search = search->get_prev_iterator();
            continue;
          }

          vector3_type diff = p - search->get_origin_iterator()->m_coord;
          vector3_type u;
          compute_edge_direction(*search,u);
          u /= std::sqrt(u*u);
          vector3_type ortho = diff - u*(u*diff);
          return std::sqrt(ortho*ortho);
        }
        while(forever);
      }
      //--- Now we know that point is "inside" the face
      mesh::compute_face_plane(f,plane);
      return plane.signed_distance(p);
    }

    template<typename mesh_type,typename vector3_type>
    typename vector3_type::value_type
      compute_distance(PolyMeshEdge<mesh_type> const & e,vector3_type const & p)
    {
      typedef typename mesh_type::vertex_iterator     vertex_iterator;
      typedef typename mesh_type::halfedge_iterator   halfedge_iterator;
      typedef typename mesh_type::face_iterator       face_iterator;

      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::real_type                 real_type;
      typedef geometry::Plane<math_types>                    plane_type;

      plane_type plane;

      halfedge_iterator h0 = e.get_halfedge0_iterator();
      vertex_iterator   v0 = h0.get_destination_iterator();
      compute_vertex_edge_voronoi_plane(v0,h0,plane);
      if(plane.signed_distance(p)>0)
        return compute_distance(v0,p);

      halfedge_iterator h1 = e.get_halfedge1_iterator();
      vertex_iterator   v1 = h1.get_destination_iterator();
      compute_vertex_edge_voronoi_plane(v1,h1,plane);
      if(plane.signed_distance(p)>0)
        return compute_distance(v1,p);

      face_iterator     f0 = h0.get_face_iterator();
      compute_edge_face_voronoi_plane(h0,f0,plane);
      if(plane.signed_distance(p)>0)
        return compute_distance(f0,p);

      face_iterator     f1 = h1.get_face_iterator();
      compute_edge_face_voronoi_plane(h1,f1,plane);
      if(plane.signed_distance(p)>0)
        return compute_distance(f1,p);

      vector3_type u = v0->m_coord - v1->m_coord;
      vector3_type delta = p - v1->m_coord;
      real_type proj = u * delta;
      vector3_type orto = delta - u*proj;
      return std::sqrt(orto*orto);
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_COMPUTE_DISTANCE_H
#endif
