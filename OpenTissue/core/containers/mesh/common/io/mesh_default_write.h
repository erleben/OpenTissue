#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_DEFAULT_WRITE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_DEFAULT_WRITE_H
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
    * Write default opentissue mesh (.msh) file.
    *
    *
    * @param filename
    * @param mesh
    *
    * @return
    */
    template<typename mesh_type>
    bool default_write(std::string const & filename, mesh_type const & mesh)
    {
      typedef typename mesh_type::math_types                    math_types;
      typedef typename math_types::value_traits                 value_traits;
      typedef typename math_types::vector3_type                 vector3_type;
      typedef typename math_types::real_type                    real_type;
      typedef typename mesh_type::vertex_handle                 vertex_handle;
      typedef typename mesh_type::index_type                    index_type;
      typedef typename mesh_type::const_vertex_iterator         const_vertex_iterator;
      typedef typename mesh_type::const_face_iterator           const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator  const_face_vertex_circulator;

      std::ofstream file(filename.c_str());
      file.precision(30);

      file << "NODES\n{\n";
      const_vertex_iterator vend = mesh.vertex_end();
      const_vertex_iterator v    = mesh.vertex_begin();

      index_type idx = 0;
      for(;v!=vend;++v,++idx)
      {
        assert(v->get_handle().get_idx() == idx || !"Oh no, vertices did not have consecutive indices???");
        file << '\t' << v->m_coord(0)
          << '\t' << v->m_coord(1)
          << '\t' << v->m_coord(2)
          << '\n';
      }
      file << "}\n";

      file << "FACES\n{\n";
      const_face_iterator fend = mesh.face_end();
      const_face_iterator f    = mesh.face_begin();

      for(;f!=fend;++f)
      {
        if(valency(*f)!=3)
        {
          std::cout << "mesh::default_write(): Warning, non-triangular face with " << valency(*f) << " vertices" << std::endl;
          continue;
        }
        const_face_vertex_circulator fvc(*f),end;
        for(;fvc!=end;++fvc)
        {
          file << '\t' << fvc->get_handle().get_idx();
        }
        file << '\n';
      }
      file << "}\n";
      std::cout << "mesh::default_write(...): written " << filename << std::endl;
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_DEFAULT_WRITE_H
#endif
