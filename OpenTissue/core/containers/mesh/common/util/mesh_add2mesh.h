#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_ADD2MESH_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_ADD2MESH_H
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
     * Add Mesh to Another Mesh.
     *
     * @param A    A reference to the input mesh.
     * @param C    Upon return the geometry of mesh A will have been added to this mesh.
     *
     * @return     If operation is succesful then the return value is
     *             true otherwise it is false.
     */
    template<typename mesh_type>
    bool add2mesh(
        mesh_type const & A
      , mesh_type & C
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

      const_vertex_iterator vend = A.vertex_end();
      const_vertex_iterator v    = A.vertex_begin();
      for(;v!=vend;++v)
      {
        vertex_handle h = C.add_vertex();
        assert(!h.is_null() || !"add2mesh(): Internal error, Could not create vertex in output mesh");
        vh_lut[v->get_handle().get_idx()] = h;
        vertex_iterator V = C.get_vertex_iterator(h);

        vertex_traits * Vt = static_cast<vertex_traits*>(&(*V));
        vertex_traits const * vt  = static_cast<vertex_traits const *>(&(*v));
        (*Vt) = (*vt);
      }
      const_face_iterator fend = A.face_end();
      const_face_iterator f    = A.face_begin();
      for(;f!=fend;++f)
      {
        std::list<vertex_handle> handles;
        const_face_vertex_circulator vc(*f),vcend;
        for(;vc!=vcend;++vc)
        {
          vertex_handle h = vh_lut[vc->get_handle().get_idx()];
          assert(!h.is_null() || !"add2mesh(): Internal error, could not find vertices in output mesh");
          handles.push_back(h);
        }
        face_handle h = C.add_face(handles.begin(),handles.end());
        assert(!h.is_null() || !"add2mesh(): Internal error, Could not create face in output mesh");
        face_iterator F = C.get_face_iterator(h);

        face_traits * Ft = static_cast<face_traits*>(&(*F));
        face_traits const * ft  = static_cast<face_traits const *>(&(*f));
        (*Ft) = (*ft);
      }
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_ADD2MESH_H
#endif
