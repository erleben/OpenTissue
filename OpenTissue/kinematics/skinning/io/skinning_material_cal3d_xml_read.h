#ifndef OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_MATERIAL_CAL3D_XML_READ_H
#define OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_MATERIAL_CAL3D_XML_READ_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <TinyXML/tinyxml.h>

#include <cassert>
#include <string>
#include <sstream>
#include <iostream>

namespace OpenTissue
{
  namespace skinning
  {

    template<typename material_type>
    bool material_cal3d_xml_read(std::string const & filename, material_type & mat)
    {

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
      TiXmlElement * xml_material = xml_document.FirstChildElement( "MATERIAL" );
      if(xml_material)
      {
        TiXmlElement * ambient = xml_material->FirstChildElement("AMBIENT");
        if(ambient)
        {
          float r,g,b,a;

          node = ambient->FirstChild();
          TiXmlText* ambientdata = node->ToText();
          str.clear();
          str << ambientdata->Value();
          str >> r >> g >> b >> a;

          if(r>1.0)  r /= 255.0;
          if(g>1.0)  g /= 255.0;
          if(b>1.0)  b /= 255.0;
          if(a>1.0)  a /= 255.0;
          mat.ambient(r,g,b,a);
        }

        TiXmlElement * diffuse = xml_material->FirstChildElement("DIFFUSE");

        if(diffuse)
        {
          float r,g,b,a;

          node = diffuse->FirstChild();
          TiXmlText* diffusedata = node->ToText();
          str.clear();
          str << diffusedata->Value();
          str >> r >> g >> b >> a;

          if(r>1.0)  r /= 255.0;
          if(g>1.0)  g /= 255.0;
          if(b>1.0)  b /= 255.0;
          if(a>1.0)  a /= 255.0;
          mat.diffuse(r,g,b,a);
        }

        TiXmlElement * specular = xml_material->FirstChildElement("SPECULAR");

        if(specular)
        {
          float r,g,b,a;

          node = specular->FirstChild();
          TiXmlText* speculardata = node->ToText();
          str.clear();
          str << speculardata->Value();
          str >> r >> g >> b >> a;

          if(r>1.0)  r /= 255.0;
          if(g>1.0)  g /= 255.0;
          if(b>1.0)  b /= 255.0;
          if(a>1.0)  a /= 255.0;

          mat.specular(r,g,b,a);
        }

        TiXmlElement * shininess = xml_material->FirstChildElement("SHININESS");

        if(shininess)
        {
          float s;

          node = shininess->FirstChild();
          TiXmlText* shininessdata = node->ToText();
          str.clear();
          str << shininessdata->Value();
          str >> s;

          mat.shininess(s);
        }
      }

      xml_document.Clear();
      return true;
    };

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_MATERIAL_CAL3D_XML_READ_H
#endif
