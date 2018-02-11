#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_VERTEX_EXPAND_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_VERTEX_EXPAND_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh_vertex.h>

namespace OpenTissue
{
  namespace polymesh
  {

    /**
    * Expand Vertex into Faces.
    *
    * Warning indices and handles of affected primitives may not be preserved!!!
    *
    *
    * @param v
    *
    * @return
    */
    template<typename mesh_type>
    bool vertex_expand(PolyMeshVertex<mesh_type> & v)
    {
      typedef typename mesh_type::vertex_halfedge_circulator      vertex_halfedge_circulator;
      typedef typename mesh_type::halfedge_type                   halfedge_type;
      typedef typename mesh_type::vertex_handle                   vertex_handle;
      typedef typename mesh_type::face_handle                     face_handle;

      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      typedef typename math_types::real_type                        real_type;

      mesh_type * owner = v.get_owner();
      if(owner==0)
      {
        assert(!"face_collapse(): No mesh owner!");
        return false;
      }
      int i,j;
      int n = valency(v);

      typedef std::vector<vertex_handle> boundary_container;


      boundary_container handles(n);
      {
        vertex_halfedge_circulator h(v),hend;
        for(i=0;h!=hend;++h,++i)
        {
          vector3_type midpoint = ( h->get_origin_iterator()->m_coord + h->get_destination_iterator()->m_coord )*.5;
          handles[i] = owner->add_vertex(midpoint);
        }
      }

      std::list<face_handle> neighbors;
      std::vector<boundary_container> boundaries(n);
      {
        vertex_halfedge_circulator cur(v);
        vertex_halfedge_circulator next(v);++next;
        for(i=0,j=1; i<n; ++i,j=(j+1)%n,++cur,++next)
        {
          if(!cur->get_face_handle().is_null())
          {
            neighbors.push_back( cur->get_face_handle() );
            boundaries[i].push_back( handles[j] );
            halfedge_type * loop = &(*next);
            halfedge_type * stop = &(*cur->get_twin_iterator());
            do
            {
              boundaries[i].push_back( loop->get_destination_handle() );
              loop = &(*(loop->get_next_iterator()));
            }while(loop != stop );
            boundaries[i].push_back( handles[i] );
          }
        }
      }

      {
        for(typename std::list<face_handle>::iterator f = neighbors.begin();f!=neighbors.end();++f)
          owner->remove_face( *f );
        owner->remove_vertex( v.get_handle() );
      }

      for(i=0;i<n;++i)
      {
        if(!boundaries[i].empty())
          owner->add_face(boundaries[i].begin(),boundaries[i].end());
      }

      owner->add_face(handles.rbegin(),handles.rend());

      return true;
    }

  } // namespace polymesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_POLYMESH_UTIL_POLYMESH_VERTEX_EXPAND_H
#endif
