#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_REFLEX_CONVEX_DECOMPOSITION_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_REFLEX_CONVEX_DECOMPOSITION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/common/util/mesh_plane_clipper.h>

#include <OpenTissue/core/geometry/geometry_plane.h> //--- needed for plane_type

namespace OpenTissue
{
  namespace polymesh
  {

    template<  typename mesh_type, typename mesh_container >
    bool reflex_convex_decomposition( mesh_type & mesh,  mesh_container & pieces)
    {
      typedef typename mesh_type::halfedge_iterator halfedge_iterator;
      typedef typename mesh_type::halfedge_type     halfedge_type;
      typedef typename mesh_type::vertex_iterator   vertex_iterator;
      typedef typename mesh_type::vertex_handle     vertex_handle;
      typedef typename mesh_type::face_iterator     face_iterator;
      typedef typename mesh_type::index_type        index_type;

      typedef typename mesh_type::math_types                 math_types;
      typedef typename math_types::vector3_type              vector3_type;
      typedef typename math_types::real_type                 real_type;
      typedef geometry::Plane<math_types>                    plane_type;

      pieces.clear();

      std::list<mesh_type *> processing;
      processing.push_back( &mesh );

      while(!processing.empty())
      {
        mesh_type * piece = processing.back();processing.pop_back();

        bool reflex_edge_found = false;

        halfedge_iterator h = piece->halfedge_begin();
        halfedge_iterator hend = piece->halfedge_end();
        for(;h!=hend;++h)
        {
          if(is_boundary(*(h->get_edge_iterator())))
            continue;

          if(is_concave(*h))
          {
            reflex_edge_found = true;

            mesh_type * part1 = new mesh_type();
            mesh_type * part2 = new mesh_type();

            plane_type Q;

            face_iterator   f0          = h->get_face_iterator();
            face_iterator   f1          = h->get_twin_iterator()->get_face_iterator();
            vertex_iterator origin      = h->get_origin_iterator();
            vertex_iterator destination = h->get_destination_iterator();

            vector3_type n0,n1,u,n;
            compute_face_normal(*f0,n0);
            compute_face_normal(*f1,n1);
            u = destination->m_coord - origin->m_coord;
            n = (n0+n1) % u;
            Q.set(n,origin->m_coord);

            mesh::plane_clipper(*piece,Q,*part1,*part2);

            if(piece != &mesh)
              delete piece;

            if(part1->size_faces()>0)
            {
              //naive_patcher(*part1);
              processing.push_back(part1);
            }
            else
            {
              delete part1;
            }
            if(part2->size_faces()>0)
            {
              //naive_patcher(*part2);
              processing.push_back(part2);
            }
            else
            {
              delete part2;
            }
            break;
          }
        }

        if(!reflex_edge_found)
        {
          pieces.push_back( *piece);
          if(piece != &mesh)
            delete piece;
        }
      }

      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_REFLEX_CONVEX_DECOMPOSITION_H
#endif
