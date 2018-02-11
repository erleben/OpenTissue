#ifndef OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_CAL3D_XML_READ_H
#define OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_CAL3D_XML_READ_H
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


    /**
    *
    * @param skin_lst     A list of skin meshes.
    */
    template<typename skin_type_lst>
    bool cal3d_xml_read(std::string const & filename, skin_type_lst & skin_lst)
    {
      typedef typename skin_type_lst::value_type              value_type;
      typedef typename value_type::vector3_type               vector3_type;
      typedef typename value_type::real_type                  real_type;
      typedef typename value_type::vertex_handle              vertex_handle;
      typedef typename value_type::vertex_iterator            vertex_iterator;
      typedef typename value_type::face_handle                face_handle;

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

      std::stringstream str;
      TiXmlNode* node;
      TiXmlElement * xml_skin = xml_document.FirstChildElement( "MESH" );
      TiXmlElement * xml_mesh = xml_skin->FirstChildElement();

      for( ; xml_mesh; xml_mesh=xml_mesh->NextSiblingElement())
      {
        //skin_type & skin = skin_lst[si];
        value_type skin;
        skin.clear();

        if(xml_mesh)
        {
          skin.m_material_idx = -1;
          if(xml_mesh->Attribute("MATERIAL"))
          {
            std::istringstream str_stream(xml_mesh->Attribute("MATERIAL"));
            str_stream >> skin.m_material_idx;
          }
        }

        TiXmlElement * xml_vertex = xml_mesh->FirstChildElement( "VERTEX" );

        for( ; xml_vertex; xml_vertex=xml_vertex->NextSiblingElement("VERTEX") )
        {
          vertex_handle h = skin.add_vertex();
          assert(!h.is_null() || !"cal3d_xml_read(): Could not create vertex");
          vertex_iterator vertex = skin.get_vertex_iterator(h);
          if(xml_vertex->Attribute("idx"))
          {
            unsigned int idx;
            std::istringstream str_stream(xml_vertex->Attribute("ID"));
            str_stream >> idx;
            if(h.get_idx()!=idx)
            {
              std::cerr << "cal3d_xml_read(): Error in vertex indices" << std::endl;
            }
          }

          TiXmlElement * xml_pos = xml_vertex->FirstChildElement( "POS" );
          float tx, ty, tz;

          if( xml_pos )
          {
            node = xml_pos->FirstChild();
            TiXmlText* positiondata = node->ToText();
            str.clear();
            str << positiondata->Value();
            str >> tx >> ty >> tz;

            vertex->m_coord(0) = tx;
            vertex->m_coord(1) = ty;
            vertex->m_coord(2) = tz;
          }

          TiXmlElement * xml_norm = xml_vertex->FirstChildElement( "NORM" );
          float nx, ny, nz;

          if(  xml_norm )
          {
            node = xml_norm->FirstChild();
            TiXmlText* normaldata = node->ToText();
            str.clear();
            str << normaldata->Value();
            str >> nx >> ny >> nz;

            vertex->m_normal(0) = nx;
            vertex->m_normal(1) = ny;
            vertex->m_normal(2) = nz;
          }

          vertex->m_original_coord = vertex->m_coord;
          vertex->m_original_normal = vertex->m_normal;

          vertex->m_influences = 0;
          if(xml_vertex->Attribute("NUMINFLUENCES"))
          {
            std::istringstream str_stream(xml_vertex->Attribute("NUMINFLUENCES"));
            str_stream >> vertex->m_influences;
          }


          TiXmlElement * xml_influence = xml_vertex->FirstChildElement( "INFLUENCE" );
          for(unsigned int cnt = 0 ; xml_influence; xml_influence=xml_influence->NextSiblingElement("INFLUENCE"),++cnt )
          {
            if(cnt>=6)
            {
              std::cout << "cal3d_xml_read(): exceeded available space for influences" << std::endl;
              break;
            }
            vertex->m_bone[cnt] = -1;
            vertex->m_weight[cnt] = 0;

            if(xml_influence->Attribute("ID"))
            {
              std::istringstream str_stream(xml_influence->Attribute("ID"));
              str_stream >> vertex->m_bone[cnt];
            }

            float weight = 0;

            node = xml_influence->FirstChild();
            TiXmlText* weightdata = node->ToText();
            str.clear();
            str << weightdata->Value();
            str >> weight;
            vertex->m_weight[cnt] = weight;
          }
        }

        TiXmlElement * xml_face = xml_mesh->FirstChildElement( "FACE" );

        for( ; xml_face; xml_face=xml_face->NextSiblingElement("FACE") )
        {
          if(!xml_face->Attribute("VERTEXID"))
          {
            std::cerr << "xml_read(): Error missing vertices in face tag" << std::endl;
          }
          std::istringstream str_stream(xml_face->Attribute("VERTEXID"));
          std::list<vertex_handle> handles;
          while(!str_stream.eof())
          {
            unsigned int idx;
            str_stream >> idx;
            vertex_handle vh = skin.get_vertex_handle( idx );
            assert(!vh.is_null() || !"cal3d_xml_read(): Could not find vertex");
            handles.push_back( vh  );
          }
          face_handle h = skin.add_face(handles.begin(),handles.end());
          assert(!h.is_null() || !"cal3d_xml_read() : Internal error, could not create face");
        }

        skin_lst.push_back( skin );

      }

      //TiXmlHandle document_handle( &xml_document );

      //TiXmlElement * xml_mesh = document_handle.FirstChild( "mesh" ).Element();
      xml_document.Clear();
      return true;
    }

  } // namespace skinning

} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_CAL3D_XML_READ_H
#endif
