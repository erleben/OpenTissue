#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_OBJ_WRITE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_OBJ_WRITE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>

namespace OpenTissue
{
  namespace mesh
  {
    /**
    * Write OBJ mesh (.obj) file.
    *
    *
    * @param filename
    * @param mesh
    *
    * @return
    */
    template<typename mesh_type>
    bool obj_write(std::string const & filename, mesh_type const & mesh)
    {
      typedef typename mesh_type::index_type                    index_type;
      typedef typename mesh_type::const_vertex_iterator         const_vertex_iterator;
      typedef typename mesh_type::const_face_iterator           const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator  const_face_vertex_circulator;

      std::ofstream file(filename.c_str());

      if(!file.is_open())
        return false;

      file << "g group0"  << std::endl;

      file.precision(30);
      const_vertex_iterator vend = mesh.vertex_end();
      const_vertex_iterator v    = mesh.vertex_begin();
      for(;v!=vend;++v)
      {
        file << "v " << v->m_coord(0) << ' ' << v->m_coord(1)  << ' ' << v->m_coord(2)  << std::endl;
        file << "vt " << v->m_u << ' ' << v->m_v << std::endl;
        file << "vn " << v->m_normal(0) << ' ' << v->m_normal(1)  << ' ' << v->m_normal(2)  << std::endl;
      }
      const_face_iterator fend = mesh.face_end();
      const_face_iterator f    = mesh.face_begin();
      for(;f!=fend;++f)
      {
        file << "f";
        const_face_vertex_circulator fvc(*f),end;
        for(;fvc!=end;++fvc)
        {
          index_type idx = fvc->get_handle().get_idx() + 1; //--- need one-based indices!!!
          file << " " <<  idx << "/"   << idx << "/"   << idx;
          //file << " " <<  idx;
        }
        file << std::endl;
      }
      file.flush();
      file.close();
      std::cout << "obj_write(...): written " << filename << std::endl;
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_OBJ_WRITE_H
#endif
