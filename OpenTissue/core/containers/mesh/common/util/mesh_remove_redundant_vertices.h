#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_MESH_UTIL_MESH_REMOVE_REDUNDANT_VERTICES_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_MESH_UTIL_MESH_REMOVE_REDUNDANT_VERTICES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace mesh
  {

    /**
    * Quadratic Time Brute force removeal of redundant vertices.
    *
    * This function searches the input mesh for vertices with sufficiently
    * similar coordinates and collapses these into a single vertex in
    * the output mesh.
    *
    *
    * @param input
    * @param output
    */
    template< typename mesh_type >
    void remove_redundant_vertices( mesh_type /*const*/ & input, mesh_type & output, double tolerance = 10e-7 )
    {
      typedef typename mesh_type::math_types                        math_types;
      typedef typename math_types::value_traits                     value_traits;
      typedef typename math_types::vector3_type                     vector3_type;
      typedef typename math_types::real_type                        real_type;

      output.clear();

      std::vector<typename mesh_type::vertex_handle> lut;

      typename mesh_type::vertex_iterator vbegin = input.vertex_begin();
      typename mesh_type::vertex_iterator vend   = input.vertex_end();
      typename mesh_type::vertex_iterator v      = vbegin;
      unsigned int i=0;
      for(;v!=vend;++v,++i)
      {
        v->m_tag = i;
        typename mesh_type::vertex_iterator w  = vbegin;
        bool redundant_vertex = false;
        unsigned int j = 0;
        for(;w!=v;++w,++j)
        {
          vector3_type diff = v->m_coord - w->m_coord;
          if(diff*diff<tolerance)
          {
            redundant_vertex = true;
            break;
          }
        }
        if(redundant_vertex)
        {
#ifndef NDEBUG
          std::cout << "remove_redundant_vertices(): redundant vertex detected" << std::endl;
#endif
          lut.push_back(lut[j]);
        }
        else
        {
          typename mesh_type::vertex_handle h = output.add_vertex( v->m_coord );
          lut.push_back(h);
        }
      }

      typename mesh_type::face_iterator fbegin = input.face_begin();
      typename mesh_type::face_iterator fend   = input.face_end();
      typename mesh_type::face_iterator f      = fbegin;
      for(;f!=fend;++f)
      {
        std::list<typename mesh_type::vertex_handle> handles;
        typename mesh_type::face_vertex_circulator v(*f),vend;
        for(;v!=vend;++v)
          handles.push_back(lut[v->m_tag]);

        typename mesh_type::face_handle fh = output.add_face(handles.begin(),handles.end());

        if(fh.is_null())
        {
          std::cout << "remove_redundant_vertices(): could not create face" << std::endl;
        }
      }

    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_MESH_UTIL_MESH_REMOVE_REDUNDANT_VERTICES_H
#endif
