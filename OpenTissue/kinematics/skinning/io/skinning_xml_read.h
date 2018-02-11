#ifndef OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_XML_READ_H
#define OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_XML_READ_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <TinyXML/tinyxml.h>

#include <list>
#include <cassert>
#include <string>
#include <sstream>
#include <iostream>

namespace OpenTissue
{
  namespace skinning
  {

    template<typename skin_type>
    bool xml_read(std::string const & filename,skin_type & skin)
    {
      typedef typename skin_type::vector3_type                vector3_type;
      typedef typename skin_type::real_type                   real_type;
      typedef typename skin_type::vertex_handle               vertex_handle;
      typedef typename skin_type::vertex_iterator             vertex_iterator;
      typedef typename skin_type::face_handle                 face_handle;
      skin.clear();
#ifdef TIXML_USE_STL
      TiXmlDocument xml_document(filename);
#else
      TiXmlDocument xml_document(filename.c_str());
#endif
      if(!xml_document.LoadFile())
      {
        std::cerr << "file: " << filename << " not found" << std::endl;
        return false;
      }
      TiXmlHandle document_handle( &xml_document );

      TiXmlElement * xml_mesh = document_handle.FirstChild( "mesh" ).Element();
      if(xml_mesh)
      {
        skin.m_material_idx = -1;
        if(xml_mesh->Attribute("material"))
        {
          std::istringstream str_stream(xml_mesh->Attribute("material"));
          str_stream >> skin.m_material_idx;
        }
      }
      TiXmlElement * xml_vertex = document_handle.FirstChild( "mesh" ).FirstChild( "vertex" ).Element();
      for( ; xml_vertex; xml_vertex=xml_vertex->NextSiblingElement("vertex") )
      {
        vertex_handle h = skin.add_vertex();
        assert(!h.is_null() || !"xml_read(): Could not create vertex");
        vertex_iterator vertex = skin.get_vertex_iterator(h);
        if(xml_vertex->Attribute("idx"))
        {
          unsigned int idx;
          std::istringstream str_stream(xml_vertex->Attribute("idx"));
          str_stream >> idx;
          if(h.get_idx()!=idx)
          {
            std::cerr << "xml_read(): Error in vertex indices" << std::endl;
          }
        }
        if(xml_vertex->Attribute("p"))
        {
          vector3_type p;
          std::istringstream str_stream(xml_vertex->Attribute("p"));
          str_stream >> p;
          vertex->m_coord(0) = p(0);
          vertex->m_coord(1) = p(1);
          vertex->m_coord(2) = p(2);
        }
        if(xml_vertex->Attribute("n"))
        {
          vector3_type n;
          std::istringstream str_stream(xml_vertex->Attribute("n"));
          str_stream >> n;
          vertex->m_normal(0) = n(0);
          vertex->m_normal(1) = n(1);
          vertex->m_normal(2) = n(2);
        }
        if(xml_vertex->Attribute("c"))
        {
          vector3_type c;
          std::istringstream str_stream(xml_vertex->Attribute("c"));
          str_stream >> c;
          vertex->m_color(0) = c(0);
          vertex->m_color(1) = c(1);
          vertex->m_color(2) = c(2);
        }
        if(xml_vertex->Attribute("u"))
        {
          real_type u;
          std::istringstream str_stream(xml_vertex->Attribute("u"));
          str_stream >> u;
          vertex->m_u = u;
        }
        if(xml_vertex->Attribute("v"))
        {
          real_type v;
          std::istringstream str_stream(xml_vertex->Attribute("v"));
          str_stream >> v;
          vertex->m_v = v;
        }

        vertex->m_original_coord = vertex->m_coord;
        vertex->m_original_normal = vertex->m_normal;

        vertex->m_influences = 0;
        if(xml_vertex->Attribute("influences"))
        {
          std::istringstream str_stream(xml_vertex->Attribute("influences"));
          str_stream >> vertex->m_influences;
        }


        // TODO: fix - only 4...
        for(unsigned int i = 0; i<4; ++i)
        {
          vertex->m_bone[i] = -1;
          vertex->m_weight[i] = 0;
        }

        TiXmlElement * xml_influence = xml_vertex->FirstChildElement( "influence" );
        for(unsigned int cnt = 0 ; xml_influence; xml_influence=xml_influence->NextSiblingElement("influence"),++cnt )
        {
          if(cnt>=6)
          {
            std::cout << "xml_read(): exceeded available space for influences" << std::endl;
            break;
          }
          //vertex->m_bone[cnt] = -1;
          //vertex->m_weight[cnt] = 0;

          if(xml_influence->Attribute("bone"))
          {
            std::istringstream str_stream(xml_influence->Attribute("bone"));
            str_stream >> vertex->m_bone[cnt];
          }
          if(xml_influence->Attribute("weight"))
          {
            std::istringstream str_stream(xml_influence->Attribute("weight"));
            str_stream >> vertex->m_weight[cnt];
          }
        }      
      }
      TiXmlElement * xml_face = document_handle.FirstChild( "mesh" ).FirstChild( "face" ).Element();
      for( ; xml_face; xml_face=xml_face->NextSiblingElement("face") )
      {
        if(!xml_face->Attribute("vertices"))
        {
          std::cerr << "xml_read(): Error missing vertices in face tag" << std::endl;
        }
        std::istringstream str_stream(xml_face->Attribute("vertices"));
        std::list<vertex_handle> handles;
        while(!str_stream.eof())
        {
          unsigned int idx;
          str_stream >> idx;
          vertex_handle vh = skin.get_vertex_handle( idx );
          assert(!vh.is_null() || !"xml_read(): Could not find vertex");
          handles.push_back( vh  );
        }
        face_handle h = skin.add_face(handles.begin(),handles.end());
        assert(!h.is_null() || !"xml_read() : Internal error, could not create face");
      }
      xml_document.Clear();
      return true;
    }

  } // namespace skinning

} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_XML_READ_H
#endif
