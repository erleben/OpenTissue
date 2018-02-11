#ifndef OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_MATERIAL_XML_READ_H
#define OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_MATERIAL_XML_READ_H
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
    bool material_xml_read(std::string const & filename,material_type & mat)
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
      TiXmlHandle document_handle( &xml_document );

      TiXmlElement * xml_material = document_handle.FirstChild( "material" ).Element();
      if(xml_material)
      {
        if(xml_material->Attribute("ambient"))
        {
          float r,g,b,a;
          std::istringstream str_stream(xml_material->Attribute("ambient"));
          str_stream >> r;      if(r>1.0)  r /= 255.0;
          str_stream >> g;      if(g>1.0)  g /= 255.0;
          str_stream >> b;      if(b>1.0)  b /= 255.0;
          str_stream >> a;      if(a>1.0)  a /= 255.0;
          mat.ambient(r,g,b,a);
        }
        if(xml_material->Attribute("diffuse"))
        {
          float r,g,b,a;
          std::istringstream str_stream(xml_material->Attribute("diffuse"));
          str_stream >> r;      if(r>1.0)  r /= 255.0;
          str_stream >> g;      if(g>1.0)  g /= 255.0;
          str_stream >> b;      if(b>1.0)  b /= 255.0;
          str_stream >> a;      if(a>1.0)  a /= 255.0;
          mat.diffuse(r,g,b,a);
        }
        if(xml_material->Attribute("specular"))
        {
          float r,g,b,a;
          std::istringstream str_stream(xml_material->Attribute("specular"));
          str_stream >> r;      if(r>1.0)  r /= 255.0;
          str_stream >> g;      if(g>1.0)  g /= 255.0;
          str_stream >> b;      if(b>1.0)  b /= 255.0;
          str_stream >> a;      if(a>1.0)  a /= 255.0;
          mat.specular(r,g,b,a);
        }
        if(xml_material->Attribute("shininess"))
        {
          float s;
          std::istringstream str_stream(xml_material->Attribute("shininess"));
          str_stream >> s;
          mat.shininess(s);
        }
      }
      xml_document.Clear();
      return true;
    };

  } // namespace skinning
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKINNING_IO_SKINNING_MATERIAL_XML_READ_H
#endif
