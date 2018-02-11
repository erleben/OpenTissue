#ifndef OPENTISSUE_KINEMATICS_ANIMATION_IO_ANIMATION_KEYFRAME_ANIMATION_XML_READ_H
#define OPENTISSUE_KINEMATICS_ANIMATION_IO_ANIMATION_KEYFRAME_ANIMATION_XML_READ_H
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
     * Read XML keyframe animation.
     *
     * @param filename
     * @param keyframe_animation
     *
     * @return
     */
    template<typename keyframe_animation_type>
    bool keyframe_animation_xml_read(std::string const & filename, keyframe_animation_type & keyframe_animation)
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
      TiXmlHandle document_handle( &xml_document );

      TiXmlElement * xml_transform = document_handle.FirstChild( "animation" ).FirstChild( "transform" ).Element();
      for( ; xml_transform; xml_transform=xml_transform->NextSiblingElement("transform") )
      {
        channels_type * transform = keyframe_animation.create_joint_channels();
        if(!xml_transform->Attribute("bone"))
        {
          std::cerr << "Invalid file format" << std::endl;
          return false;
        }
        size_t  bone_number = atoi(xml_transform->Attribute("bone"));
        transform->set_bone_number(bone_number);

        TiXmlHandle transform_handle( xml_transform );
        TiXmlElement * xml_keyframe = transform_handle.FirstChild( "keyframe" ).Element();
        for( ; xml_keyframe; xml_keyframe=xml_keyframe->NextSiblingElement("keyframe") )
        {
          TiXmlHandle keyframe_handle( xml_keyframe );

          real_type time = 0;

          if(xml_keyframe->Attribute("time"))
            time  = (real_type) atof(xml_keyframe->Attribute("time"));

          if(xml_keyframe->Attribute("value"))
          {
            coordsys_type value;
            std::istringstream str_stream(xml_keyframe->Attribute("value"));
            str_stream >> value;
            transform->add_key(time,value);
          }
        }
      }
      xml_document.Clear();
      return true;
    };

  } // namespace animation
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_ANIMATION_IO_ANIMATION_KEYFRAME_ANIMATION_XML_READ_H
#endif
