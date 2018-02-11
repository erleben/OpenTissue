#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_VRML_WRITE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_VRML_WRITE_H
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
    * Write VRML mesh (.vrml) file.
    *
    *
    * @param filename
    * @param mesh
    *
    * @return
    */
    template<typename mesh_type>
    bool vrml_write(std::string const & filename, mesh_type const & mesh)
    {
      typedef typename mesh_type::index_type                    index_type;
      typedef typename mesh_type::const_vertex_iterator         const_vertex_iterator;
      typedef typename mesh_type::const_face_iterator           const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator  const_face_vertex_circulator;

      std::ofstream file(filename.c_str());
      if(!file.is_open())
        return false;
      file.precision(30);
      file << "#VRML V2.0 utf8" << std::endl << std::endl;;
      file << "Transform {" << std::endl;
      file << " children [" << std::endl;
      file << "  Shape {"<< std::endl;
      file << "   appearance Appearance {"<< std::endl;
      file << "    material Material {"<< std::endl;
      file << "     diffuseColor 0.3333 0.1098 0.6941"<< std::endl;
      file << "    }" << std::endl;
      file << "   }" << std::endl;
      file << "   geometry IndexedFaceSet {" << std::endl;
      file << "   coord Coordinate { point [" << std::endl;
      const_vertex_iterator vend = mesh.vertex_end();
      const_vertex_iterator v    = mesh.vertex_begin();
      for(;v!=vend;++v)
        file << "     " << v->m_coord(0) << ' ' << v->m_coord(1)  << ' ' << v->m_coord(2)  << std::endl;
      file << "    ]}" << std::endl;
      file << "    coordIndex [" << std::endl;
      const_face_iterator fend = mesh.face_end();
      const_face_iterator f    = mesh.face_begin();
      for(;f!=fend;++f)
      {
        assert(valency(*f)==3 || !"Only triangular faces are supported");
        const_face_vertex_circulator i(*f);--i;
        const_face_vertex_circulator j(*f);
        const_face_vertex_circulator k(*f);++k;
        file << "     "
          << i->get_handle().get_idx()
          << " "
          << j->get_handle().get_idx()
          << " "
          << k->get_handle().get_idx()
          << " -1"
          << std::endl;
      }
      file << "    ]" << std::endl;
      file << "   }" << std::endl;
      file << "  }" << std::endl;
      file << " ]" << std::endl;
      file << "}" << std::endl;
      file.flush();
      file.close();
      std::cout << "vrml_write(...): written " << filename << std::endl;
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_VRML_WRITE_H
#endif
