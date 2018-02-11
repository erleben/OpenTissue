#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_CONVERT_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_CONVERT_H
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
    * Mesh conversion.
    * This function converts a mesh of one type into a mesh of another type.
    *
    * Example usage:
    *
    *  typedef OpenTissue::polymesh::PolyMesh<...> mesh_type1;
    *  typedef OpenTissue::TriMesh <...> mesh_type2;
    *  typedef OpenTissue::polymesh::PolyMesh<...> mesh_type3;
    *  mesh_type1 A;
    *  mesh_type2 B;
    *  mesh_type3 C;
    *
    *  convert(A,B);
    *  convert(B,A);
    *  convert(B,C);
    *  convert(A,C);
    *  ...
    *
    * Topology is converted from one mesh representation to the ohter. Also
    * vertex and face traits are assigned to each other. Function do not
    * have any knowledge of  halfedge and edge traits.
    *
    * @param in      Reference to input mesh.
    * @param out     Reference to output mesh.
    *
    * @return        The value true if succesful otherwise false.
    */
    template<typename mesh_type_in, typename mesh_type_out>
    bool convert(   mesh_type_in const & in , mesh_type_out & out  )
    {
      typedef typename mesh_type_in::const_vertex_iterator             in_vertex_iterator;
      typedef typename mesh_type_in::const_face_iterator               in_face_iterator;
      typedef typename mesh_type_in::const_face_vertex_circulator      in_face_vertex_circulator;
      typedef typename mesh_type_in::vertex_traits                     in_vertex_traits;
      typedef typename mesh_type_in::face_traits                       in_face_traits;
      typedef typename mesh_type_in::index_type                        in_index_type;

      typedef typename mesh_type_out::vertex_iterator         out_vertex_iterator;
      typedef typename mesh_type_out::vertex_handle           out_vertex_handle;
      typedef typename mesh_type_out::vertex_traits           out_vertex_traits;
      typedef typename mesh_type_out::face_traits             out_face_traits;
      typedef typename mesh_type_out::face_iterator           out_face_iterator;
      typedef typename mesh_type_out::face_handle             out_face_handle;

      typedef std::map<in_index_type,out_vertex_handle>   vh_lut_type;
      vh_lut_type vh_lut;

      out.clear();

      in_vertex_iterator vend = in.vertex_end();
      in_vertex_iterator v    = in.vertex_begin();
      for(;v!=vend;++v)
      {
        out_vertex_handle h = out.add_vertex();
        assert(!h.is_null() || !"convert(): Internal error, Could not create vertex in output mesh");
        vh_lut[v->get_handle().get_idx()] = h;
        out_vertex_iterator o = out.get_vertex_iterator(h);

        out_vertex_traits * ot = static_cast<out_vertex_traits*>(&(*o));
        in_vertex_traits const * it  = static_cast< in_vertex_traits const *>(&(*v));
        (*ot) = (*it);
      }

      in_face_iterator fend = in.face_end();
      in_face_iterator f    = in.face_begin();
      for(;f!=fend;++f)
      {
        std::list<out_vertex_handle> handles;
        in_face_vertex_circulator vc(*f),vcend;
        for(;vc!=vcend;++vc)
        {
          out_vertex_handle h = vh_lut[vc->get_handle().get_idx()];
          assert(!h.is_null() || !"convert(): Internal error, could not find vertes in output mesh");
          handles.push_back(h);
        }
        out_face_handle h = out.add_face(handles.begin(),handles.end());
        assert(!h.is_null() || !"convert(): Internal error, Could not create face in output mesh");
        out_face_iterator o = out.get_face_iterator(h);

        out_face_traits * ot = static_cast<out_face_traits*>(&(*o));
        in_face_traits const * it  = static_cast< in_face_traits const *>(&(*f));
        (*ot) = (*it);
      }
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_UTIL_MESH_CONVERT_H
#endif
