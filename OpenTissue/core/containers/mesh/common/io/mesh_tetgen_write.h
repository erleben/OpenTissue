#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_TETGEN_WRITE_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_TETGEN_WRITE_H
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
    * Write TetGen polygon (.poly) file.
    *
    * @param filename
    * @param mesh
    *
    * @return
    */
    template<typename mesh_type>
    bool tetgen_write(std::string const & filename, mesh_type const & mesh)
    {
      typedef typename mesh_type::const_vertex_iterator         const_vertex_iterator;
      typedef typename mesh_type::const_face_iterator           const_face_iterator;
      typedef typename mesh_type::const_face_vertex_circulator  const_face_vertex_circulator;
      /*
      #Node Section
      nCnt 3 0  0             #Number of nodes, dimension, number of attributes, boundary marker
      nIdx xval yval zval     #Node index, followed by x,y and z coordinates
      */
      std::ofstream file(filename.c_str());
      file.precision(30);
      file << static_cast<int>(mesh.size_vertices()) << " 3" << std::endl;
      const_vertex_iterator vend = mesh.vertex_end();
      const_vertex_iterator v    = mesh.vertex_begin();
      for(;v!=vend;++v)
        file << v->get_handle().get_idx() << " " << v->m_coord(0) << " " << v->m_coord(1)  << " " << v->m_coord(2)  << std::endl;
      file << std::endl;

      /*
      #Face Section
      fCnt 0                #Number of faces, boundary markers
      1               #Number of polygons
      3 n1Idx n2Idx ........ nNIdx         #Number of vertices, followed by node indices
      */
      file << static_cast<int>(mesh.size_faces()) << " 0" << std::endl;
      const_face_iterator fend = mesh.face_end();
      const_face_iterator f    = mesh.face_begin();
      for(;f!=fend;++f)
      {
        file << "1" << std::endl;
        file << static_cast<int>(valency(*f));
        const_face_vertex_circulator fvc(*f),end;
        for(;fvc!=end;++fvc)
          file << " " << fvc->get_handle().get_idx();
        file << std::endl;
      }

      /*
      #Hole Section
      0                     #Number of holes
      */
      file << "0" << std::endl;
      /*
      #Region Section
      rCnt                     #Number of regions
      rIdx xval yval zval rNo rAtt             #region index, coordinates of point inside region, region number, region attribute
      */
      file << "0" << std::endl;
      file.flush();
      file.close();
      std::cout << "tetgen_write(): wrote tetgen file: " << filename << std::endl;
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_TETGEN_WRITE_H
#endif
