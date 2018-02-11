#ifndef OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_XML_READ_H
#define OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_XML_READ_H
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
  namespace skeleton
  {
    namespace detail
    {


      /**
      * Read Skeleton.
      *
      * @param document_handle   A xml handle for the xml document that is being read.
      * @param skeleton          Upon return this argument holds the skeleton that was read.
      *
      * @return           If skeleton was read succesfully then the return
      *                   value is true otherwise it is false.
      */
      template<typename skeleton_type>
      bool xml_read(TiXmlHandle & document_handle, skeleton_type & skeleton)
      {
        typedef typename skeleton_type::math_types::coordsys_type    coordsys_type;
        typedef typename skeleton_type::bone_type                    bone_type;
        typedef typename bone_type::bone_traits                      bone_traits;

        skeleton.clear();

        TiXmlElement * xml_bone = document_handle.FirstChild( "skeleton" ).FirstChild( "bone" ).Element();
        for( ; xml_bone; xml_bone=xml_bone->NextSiblingElement("bone") )
        {
          bone_type * bone = 0;

          if(xml_bone->Attribute("parent"))
          {
            std::string parent_name = xml_bone->Attribute("parent");
            bone_type * parent = skeleton.get_bone(parent_name);
            if(!parent)
            {
              std::cerr << "Could not find parent!" << std::endl;
              return false;
            }
            bone = skeleton.create_bone(parent);
          }
          else
          {
            bone = skeleton.create_bone();
          }
          if(xml_bone->Attribute("name"))
          {
            std::string name = xml_bone->Attribute("name");
            bone->set_name(name);
          }
          if(xml_bone->Attribute("bindpose"))
          {
            coordsys_type bind_pose;
            std::istringstream str_stream(xml_bone->Attribute("bindpose"));
            str_stream >> bind_pose;
            bone->bind_pose() = bone_traits::convert( bind_pose );
          }
          if(xml_bone->Attribute("bonespace"))
          {
            coordsys_type bone_space;
            std::istringstream str_stream(xml_bone->Attribute("bonespace"));
            str_stream >> bone_space;
            bone->bone_space() = bone_traits::convert( bone_space );
          }
        }

        skeleton.set_bind_pose();

        return true;
      }

    }// namespace detail

    /**
    * Read Skeleton.
    *
    * @param filename   The filename wherefrom the skeleton should be read.
    * @param skeleton   Upon return this argument holds the skeleton that was read.
    *
    * @return           If skeleton was read succesfully then the return
    *                   value is true otherwise it is false.
    */
    template<typename skeleton_type>
    bool xml_read(std::string const & filename,skeleton_type & skeleton)
    {

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

      if ( ! OpenTissue::skeleton::detail::xml_read( document_handle, skeleton ) )
        return false;

      xml_document.Clear();

      return true;
    }

  } // namespace skeleton
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_XML_READ_H
#endif
