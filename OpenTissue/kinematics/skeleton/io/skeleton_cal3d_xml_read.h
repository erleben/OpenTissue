#ifndef OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_CAL3D_READ_H
#define OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_CAL3D_READ_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <TinyXML/tinyxml.h>

#include <sstream>

namespace OpenTissue
{
  namespace skeleton
  {

    /**
     * Read Cal3D XML file.
     *
     * @param filename   The name and path of the cal3d xml file.
     * @param skeleton   Upon return this skeleton contains the read data.
     *
     * @return           If data was read succesfully then the return value
     *                   is true otherwise it is false.
     */
    template<typename skeleton_type>
    bool cal3d_xml_read(std::string const & filename,skeleton_type & skeleton)
    {
      typedef typename skeleton_type::math_types::coordsys_type               coordsys_type;
      typedef typename skeleton_type::math_types::vector3_type				        vector3_type;
      typedef typename skeleton_type::math_types::quaternion_type				      quaternion_type;
      typedef typename skeleton_type::bone_type                   bone_type;
      typedef typename bone_type::bone_traits                     bone_traits;

      skeleton.clear();

#ifdef TIXML_USE_STL
      TiXmlDocument xml_document(filename);
#else
      TiXmlDocument xml_document(filename.c_str());
#endif

      if(!xml_document.LoadFile())
      {
        std::cerr << "file not found:" << filename << std::endl;
        return false;
      }

      std::stringstream str;
      TiXmlNode* node;
      TiXmlElement * xml_skelton = xml_document.FirstChildElement( "SKELETON" );
      TiXmlElement * xml_bone = xml_skelton->FirstChildElement();

      for( ; xml_bone; xml_bone=xml_bone->NextSiblingElement() )
      {
        bone_type * bone = 0;

        TiXmlElement * parentid = xml_bone->FirstChildElement("PARENTID");

        if( parentid )
        {
          node = parentid->FirstChild();
          TiXmlText* parentiddata = node->ToText();
          std::string parent_name = parentiddata->Value();
          if( parent_name.compare( "-1" ) == 0 )
            bone = skeleton.create_bone();
          else
          {
            bone_type * parent = skeleton.get_bone( parent_name );
            if(!parent)
            {
              std::cerr << "Could not find parent!" << std::endl;
              return false;
            }
            bone = skeleton.create_bone(parent);
          }
        }
        else
        {
          std::cerr << "Missing xml attribute on parent!" << std::endl;
          return false;
        }

        if(xml_bone->Attribute("ID"))
        {
          std::string id = xml_bone->Attribute("ID");
          bone->set_name( id );
        }

        TiXmlElement * translation = xml_bone->FirstChildElement("TRANSLATION");
        TiXmlElement * rotation = xml_bone->FirstChildElement("ROTATION");

        float tx, ty, tz;
        float rx, ry, rz, s;

        float tx_bs, ty_bs, tz_bs;
        float rx_bs, ry_bs, rz_bs, s_bs;

        if( translation && rotation )
        {
          node = translation->FirstChild();
          TiXmlText* translationdata = node->ToText();
          str.clear();
          str << translationdata->Value();
          str >> tx >> ty >> tz;

          node = rotation->FirstChild();
          TiXmlText* rotationdata = node->ToText();
          str.clear();
          str << rotationdata->Value();
          str >> rx >> ry >> rz >> s;

          coordsys_type bind_pose( vector3_type( tx, ty, tz ),
            quaternion_type( s, -rx, -ry, -rz ) );
          bone->bind_pose() = bone_traits::convert( bind_pose );
        }

        TiXmlElement * translation_bs = xml_bone->FirstChildElement("LOCALTRANSLATION");

        TiXmlElement * rotation_bs = xml_bone->FirstChildElement("LOCALROTATION");

        if( translation_bs && rotation_bs )
        {
          node = translation_bs->FirstChild();
          TiXmlText* translationdata_bs = node->ToText();
          str.clear();
          str << translationdata_bs->Value();
          str >> tx_bs >> ty_bs >> tz_bs;

          node = rotation_bs->FirstChild();
          TiXmlText* rotationdata_bs = node->ToText();
          str.clear();
          str << rotationdata_bs->Value();
          str >> rx_bs >> ry_bs >> rz_bs >> s_bs;

          coordsys_type bone_space( vector3_type( tx_bs, ty_bs, tz_bs ),
            quaternion_type( s_bs, -rx_bs, -ry_bs, -rz_bs ) );
          bone->bone_space() = bone_traits::convert( bone_space );
        }
      }

      xml_document.Clear();
      skeleton.set_bind_pose();
      return true;
    };

  } // namespace skeleton
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_CAL3D_READ_H
#endif
