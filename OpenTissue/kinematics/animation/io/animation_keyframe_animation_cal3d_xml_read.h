#ifndef OPENTISSUE_KINEMATICS_ANIMATION_IO_ANIMATION_KEYFRAME_CAL3D_ANIMATION_XML_READ_H
#define OPENTISSUE_KINEMATICS_ANIMATION_IO_ANIMATION_KEYFRAME_CAL3D_ANIMATION_XML_READ_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <TinyXML/tinyxml.h>

namespace OpenTissue
{
  namespace animation
  {

    /**
     * Read Cal3D XML animation.
     *
     * @param filename
     * @param keyframe_animation
     *
     * @return
     */
    template<typename keyframe_animation_type>
    bool keyframe_animation_cal3d_xml_read(std::string const & filename, keyframe_animation_type & keyframe_animation)
    {
      typedef typename keyframe_animation_type::coordsys_type                             coordsys_type;
      typedef typename keyframe_animation_type::real_type                                 real_type;
      typedef typename keyframe_animation_type::channels_type                             channels_type;
      typedef typename coordsys_type::vector3_type                                        vector3_type;
      typedef typename coordsys_type::quaternion_type                                     quaternion_type;

#ifdef TIXML_USE_STL
      TiXmlDocument xml_document(filename);
#else
      TiXmlDocument xml_document(filename.c_str());
#endif

      if(!xml_document.LoadFile())
      {
        std::cerr << "file not found" << std::endl;
        return false;
      }

      std::stringstream str;
      TiXmlNode* node;
      TiXmlElement * xml_animation = xml_document.FirstChildElement( "ANIMATION" );
      TiXmlElement * xml_transform = xml_animation->FirstChildElement();

      for( ; xml_transform; xml_transform=xml_transform->NextSiblingElement() )
      {
        channels_type * transform = keyframe_animation.create_joint_channels();

        if(!xml_transform->Attribute("BONEID"))
        {
          std::cerr << "Invalid file format" << std::endl;
          return false;
        }

        size_t bone_number = atoi( xml_transform->Attribute("BONEID") );
        transform->set_bone_number(bone_number);

        TiXmlElement * xml_keyframe = xml_transform->FirstChildElement();
        for( ; xml_keyframe; xml_keyframe=xml_keyframe->NextSiblingElement() )
        {
          float tx, ty, tz;
          float rx, ry, rz, s;
          real_type time = 0;

          coordsys_type value;

          if( xml_keyframe->Attribute("TIME") )
            time  = (real_type) atof(xml_keyframe->Attribute("TIME"));

          TiXmlElement * translation = xml_keyframe->FirstChildElement();
          TiXmlElement * rotation = translation->NextSiblingElement();

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

            coordsys_type bind_pose;
            value.T()= vector3_type( tx, ty, tz );
            value.Q()= quaternion_type( s, -rx, -ry, -rz );
            transform->add_key(time,value);
          }
        }
      }
      xml_document.Clear();
      return true;
    };

  } // namespace animation
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_ANIMATION_IO_ANIMATION_KEYFRAME_CAL3D_ANIMATION_XML_READ_H
#endif
