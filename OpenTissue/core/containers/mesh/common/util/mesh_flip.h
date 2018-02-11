#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_FLIP_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_FLIP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <map>
#include <list>

namespace OpenTissue
{
  namespace mesh
  {


    /**
    * Mesh Flip.
    * This function flips a mesh.
    *
    * Example usage:
    *
    *  typedef OpenTissue::polymesh::PolyMesh<...> mesh_type;
    *  mesh_type A,B;
    *
    *  mesh::flip(A,B);
    *
    * Vertex and face traits are assigned to each other. Function do not
    * have any knowledge of  halfedge and edge traits.
    *
    *
    * @param in      Reference to input mesh.
    * @param out     Reference to output mesh.
    *
    * @return        The value true if succesful otherwise false.
    */
    template<typename mesh_type>
    bool flip(
      mesh_type const & mesh
      , mesh_type & flipped
      )
    {
      typedef typename mesh_type::vertex_handle                     vertex_handle;
      typedef typename mesh_type::face_handle                       face_handle;
      typedef typename mesh_type::vertex_iterator                   vertex_iterator;
      typedef typename mesh_type::face_iterator                     face_iterator;
      typedef typename mesh_type::const_vertex_iterator             const_vertex_iterator;
      typedef typename mesh_type::const_face_iterator               const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator      const_face_vertex_circulator;
      typedef typename mesh_type::vertex_traits                     vertex_traits;
      typedef typename mesh_type::face_traits                       face_traits;
      typedef typename mesh_type::index_type                        index_type;
      typedef std::map<index_type,vertex_handle>                    vh_lut_type;
      vh_lut_type vh_lut;

      flipped.clear();

      const_vertex_iterator vend = mesh.vertex_end();
      const_vertex_iterator v    = mesh.vertex_begin();
      for(;v!=vend;++v)
      {
        vertex_handle h = flipped.add_vertex();
        assert(!h.is_null() || !"convert(): Internal error, Could not create vertex in output mesh");
        vh_lut[v->get_handle().get_idx()] = h;
        vertex_iterator V = flipped.get_vertex_iterator(h);

        vertex_traits * Vt = static_cast<vertex_traits*>(&(*V));
        vertex_traits const * vt  = static_cast<vertex_traits const *>(&(*v));
        (*Vt) = (*vt);
      }
      const_face_iterator fend = mesh.face_end();
      const_face_iterator f    = mesh.face_begin();
      for(;f!=fend;++f)
      {
        std::list<vertex_handle> handles;
        const_face_vertex_circulator vc(*f),vcend;
        for(;vc!=vcend;--vc)
        {
          vertex_handle h = vh_lut[vc->get_handle().get_idx()];
          assert(!h.is_null() || !"convert(): Internal error, could not find vertices in flipped mesh");
          handles.push_back(h);
        }
        face_handle h = flipped.add_face(handles.begin(),handles.end());
        assert(!h.is_null() || !"convert(): Internal error, Could not create face in flipped mesh");
        face_iterator F = flipped.get_face_iterator(h);

        face_traits * Ft = static_cast<face_traits*>(&(*F));
        face_traits const * ft  = static_cast<face_traits const *>(&(*f));
        (*Ft) = (*ft);
      }
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_FLIP_H
#endif
