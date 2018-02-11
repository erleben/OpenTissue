#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_DEFAULT_READ_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_DEFAULT_READ_H
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
    * Read default opentissue mesh (.msh) file.
    *
    *
    * @param filename
    * @param mesh
    *
    * @return
    */
    template<typename mesh_type>
    bool default_read(std::string const & filename,mesh_type & mesh)
    {
      typedef typename mesh_type::math_types        math_types;
      typedef typename math_types::value_traits     value_traits;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename math_types::real_type        real_type;
      typedef typename mesh_type::vertex_handle     vertex_handle;
      typedef typename mesh_type::face_handle       face_handle;
      typedef typename mesh_type::index_type        index_type;

      mesh.clear();

      std::ifstream file(filename.c_str());
      if(!file.is_open())
      {
        std::cerr << "Error unable to open file '" << filename << "'" << std::endl;
        return false;
      }
      std::string token; //--- Temporary variable used to read tokens
      do
      { // 'NODES'
        file >> token;
      }
      while(!file.eof() && (token != "NODES" ));
      if(file.eof())
      {
        std::cerr << "Warning, file '" << filename << "' contains no nodes" << std::endl;
        return false;
      }
      do
      { // '{'
        file >> token;
      }
      while(!file.eof() && (token != "{" ));
      if(file.eof())
      {
        std::cerr << "Warning, NODES in file '" << filename << "' is corrupt" << std::endl;
        return false;
      }

      std::vector<vertex_handle> vertex_lut;  //--- vertex lookup table
      vector3_type coord;                     //--- temporary variable used to read in coordinates of vertices

      do
      { // {x y z} '}'
        file >> token;
        if(token != "}")
        {
          coord(0) = static_cast<real_type>( std::atof(token.c_str()) );
          if(file.eof())
          {
            std::cerr << "Warning, NODES in file '" << filename << "' is corrupt" << std::endl;
            return false;
          }
          file >> token;
          coord(1) = static_cast<real_type>( std::atof(token.c_str()) );
          if(file.eof())
          {
            std::cerr << "Warning, NODES in file '" << filename << "' is corrupt" << std::endl;
            return false;
          }
          file >> token;
          coord(2) = static_cast<real_type>( std::atof(token.c_str()) );
          vertex_handle h = mesh.add_vertex(coord);
          if(h.is_null())
          {
            std::cerr << "Could not create vertes" << std::endl;
            return false;
          }
          vertex_lut.push_back(h);
        }
      }
      while( token != "}" );

      do
      { // 'FACES'
        file >> token;
      }
      while(!file.eof() && (token != "FACES" ));
      if(file.eof())
      {
        std::cerr << "Warning, file '" << filename << "' contains no face pointers" << std::endl;
        return false;
      }
      do
      { // '{'
        file >> token;
      }
      while(!file.eof() && (token != "{" ));
      if(file.eof())
      {
        std::cerr << "Warning, FACES in file '" << filename << "' is corrupt" << std::endl;
        return false;
      }

      std::size_t N = vertex_lut.size();
      unsigned int v0,v1,v2;

      do
      {
        file >> token;
        if(token != "}")
        {
          v0 = static_cast<unsigned int>( std::atoi(token.c_str()) );
          if(file.eof())
          {
            std::cout << "Unexpected error?" << std::endl;
            return false;
          }
          file >> v1;
          if(file.eof())
          {
            std::cout << "Unexpected error?" << std::endl;
            return false;
          }
          file >> v2;

          if(v0>=N)
          {
            std::cerr << "Warning vertex index 0 of face is out of range" << std::endl;
            return false;
          }
          if(v1>=N)
          {
            std::cerr << "Warning vertex index 1 of face is out of range" << std::endl;
            return false;
          }
          if(v2>=N)
          {
            std::cerr << "Warning vertex index 2 of face is out of range" << std::endl;
            return false;
          }


          vertex_handle h0 = vertex_lut[v0];
          vertex_handle h1 = vertex_lut[v1];
          vertex_handle h2 = vertex_lut[v2];

          face_handle h = mesh.add_face(h0,h1,h2);
          if(h.is_null())
          {
            std::cerr << "Could not create face?" << std::endl;
            return false;
          }
        }
      }
      while(token != "}" );

      std::cout << "mesh::default_read() : "
        << filename
        << " nodes = "
        << mesh.size_vertices()
        << " faces = "
        << mesh.size_faces()
        << std::endl;
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_DEFAULT_READ_H
#endif
