#ifndef OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_OBJ_READ_H
#define OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_OBJ_READ_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/algorithm/string.hpp>  //--- needed for split and is_any_of function
#include <boost/lexical_cast.hpp>  //--- needed for lexical cast function

#include <cassert>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>

namespace OpenTissue
{
  namespace mesh
  {

    namespace detail
    {
      /**
      * Read OBJ Material Library.
      *
      * @param filename   The filename of the material library
      * @param materials  Upon return this container holds the read mateials.
      *
      * @return            A boolean value, if true the read was succesfully, if false some error occured.
      */
      template<typename material_container>
      bool read_obj_materials(std::string const & filename,material_container & materials)
      {
        typedef typename material_container::value_type     material_type;
        typedef float     real_type;
        materials.clear();

        std::ifstream file(filename);

        if(!file.is_open())
          return false;

        std::string line,type;
        std::string material_name;

        while(!file.eof())
        {
          getline(file,line);
          std::stringstream stream(line.c_str());
          stream >> type;

          if(line[0] == '#')
          {
            continue;
          }
          else if(type == "newmtl")
          {
            //--- if material_name not empty then
            //---    materials[material_name] = material_type(Ka,Kd,Ks,Tr,Ns,...)
            //--- end if
            stream >> material_name;
            continue;
          }
          else if(type == "Ka") //--- ambient color
          {
            real_type Ka0,Ka1,Ka2;
            stream >> Ka0;
            stream >> Ka1;
            stream >> Ka2;
          }
          else if(type == "Kd")//--- diffuse color
          {
            real_type Kd0,Kd1,Kd2;
            stream >> Kd0;
            stream >> Kd1;
            stream >> Kd2;
          }
          else if(type == "Ks") //--- specular color
          {
            real_type Ks0,Ks1,Ks2;
            stream >> Ks0;
            stream >> Ks1;
            stream >> Ks2;
          }
          else if(type == "Tr" || type == "d") //--- transparency value
          {
            real_type Tr;
            stream >> Tr;
          }
          else if(type == "Ns") //--- Shininess value
          {
            real_type Ns;
            stream >> Ns;
          }
          else
          {
            std::cout << "read_obj_materials(): skipping "<< type << std::endl;
          }
        }

        //--- if material_name not empty then
        //---    materials[material_name] = material_type(Ka,Kd,Ks,f,...)
        //--- end if

        file.close();
        std::cout << "read_obj_materials() : "
          << filename
          << " materials = "
          << materials.size()
          << std::endl;
        return true;
      }

    }//end of namespace detail

    /**
    * Read OBJ mesh (.obj) file.
    *
    * This implementation contains modifications suggested by
    * Jrgen Havsberg Seland and Emanuel Greisen to support
    * multiple texture coords and normals to be shared by
    * the same vertex.
    *
    *
    *
    * @param filename
    * @param mesh
    *
    * @return
    */
    template<typename mesh_type>
    bool obj_read(
      std::string const & filename
      , mesh_type & mesh
      , bool keep_shared_vertices = true )
    {
      typedef typename mesh_type::math_types        math_types;
      typedef typename math_types::value_traits     value_traits;
      typedef typename math_types::vector3_type     vector3_type;
      typedef typename math_types::real_type        real_type;
      typedef typename mesh_type::vertex_handle     vertex_handle;
      typedef typename mesh_type::vertex_iterator   vertex_iterator;
      typedef typename mesh_type::face_handle       face_handle;
      typedef typename mesh_type::index_type        index_type;
      typedef math::Vector3<unsigned int>           index_vector;
      typedef std::map<index_vector, vertex_handle> lut_type;
      typedef std::vector<vector3_type>             vector3_container;

      mesh.clear();

      std::ifstream file(filename.c_str());

      if(!file.is_open())
      {
        std::cerr << "obj_read - Could not load file: " << filename << std::endl;
        return false;
      }

      // typedef struct { ...  } material_type;
      // typedef std::map<std::string,material_type> material_container;
      // material_container  materials;

      std::string line,type;

      vector3_container texture_coords;
      vector3_container normals;
      vector3_container positions;
      lut_type lut;

      while(!file.eof())
      {
        if(getline(file,line)==0)
          break;

        //std::stringstream stream(line.c_str());
        std::stringstream stream(line);

        stream >> type;

        if(line[0] == '#')
        {
          continue;
        }
        else if(type == "g")
        {
          std::string group_name;
          stream >> group_name;
          // create new mesh with name=group name!!!
          continue;
        }
        else if(type == "usemtl")
        {
          std::string material_name;
          stream >> material_name;
          //material = materials[ material_name ];
          continue;
        }
        else if(type == "mtllib")
        {
          std::string material_filename;
          stream >> material_filename;
          //detail::read_obj_materials(material_filename, materials );
          continue;
        }
        else if(type == "v")
        {
          vector3_type p;
          stream >> p(0);
          stream >> p(1);
          stream >> p(2);
          positions.push_back(p);
        }
        else if(type == "vt")
        {
          vector3_type tex;
          stream >> tex(0);
          if(!stream.eof())
            stream >> tex(1);
          if(!stream.eof())
            stream >> tex(2);
          texture_coords.push_back(tex);
        }
        else if(type == "vn")
        {
          vector3_type n;
          stream >> n(0);
          stream >> n(1);
          stream >> n(2);
          normals.push_back(n);
        }
        else if(type == "f")
        {
          //--- parse:
          //---  f v0_idx//vn0_idx//vt0_idx  ...  vN_idx//vnN_idx//vtN_idx
          std::list<vertex_handle> handles;
          while(!stream.eof())
          {
            std::string component;
            stream >> component;
            // NOTE: hd 20060605 - hack to let linux version gracefully read windows-formatted files.
            if (component.empty())
              break;

            std::vector<std::string> split_vector;
            boost::algorithm::split(split_vector, component, boost::algorithm::is_any_of("/"),boost::algorithm::token_compress_on);

            assert(!split_vector.empty() || !"obj_read() : Internal error, no indices on face");

            std::size_t cnt = split_vector.size();
            std::vector<unsigned int>  indices;

            for(  std::vector<std::string>::iterator value = split_vector.begin();  value!=split_vector.end();   ++value)
              indices.push_back(boost::lexical_cast<unsigned int>(*value));

            index_vector key;
            key.clear();
            if( cnt >= 1 )
              key(0) = indices[0];
            if( cnt >= 2 && !keep_shared_vertices)
              key(1) = indices[1];
            if( cnt >= 3 && !keep_shared_vertices)
              key(2) = indices[2];

            typename lut_type::iterator lookup = lut.find(key);
            if( lookup==lut.end() )
              lut[key] = mesh.add_vertex( );

            assert(!lut[key].is_null() || !"obj_read(): Could not find vertex");

            handles.push_back( lut[key]  );
            vertex_iterator v = mesh.get_vertex_iterator( lut[key] );

            if( cnt >= 1 )
              v->m_coord  = positions[ indices[0] - 1];

            if( cnt >= 2 && !texture_coords.empty())
            {
              v->m_u     = texture_coords[ indices[1] - 1 ](0);
              v->m_v     = texture_coords[ indices[1] - 1 ](1);
            }
            else if( cnt >= 2 && !normals.empty())
            {
              v->m_normal = normals[ indices[1] - 1 ];
            }
            if( cnt >= 3 )
              v->m_normal = normals[ indices[2] - 1 ];




          }
          face_handle h = mesh.add_face(handles.begin(),handles.end());

          assert(!h.is_null() || !"obj_read() : Internal error, could not create face");

          //--- assign current material (set by a the usemtl line) to face
        }
        else
        {
          //std::cout << "read_obj_materials(): skipping "<< type << std::endl;
        }
      }
      file.close();
#ifndef NDEBUG
      std::cout << "obj_read() : "
        << filename
        << " nodes = "
        << mesh.size_vertices()
        << " faces = "
        << mesh.size_faces()
        << std::endl;
#endif
      return true;
    }

  } // namespace mesh
} // namespace OpenTissue

//OPENTISSUE_CORE_CONTAINERS_MESH_COMMON_IO_MESH_OBJ_READ_H
#endif
