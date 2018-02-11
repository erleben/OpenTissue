#ifndef OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_XML_WRITE_H
#define OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_XML_WRITE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <TinyXML/tinyxml.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace skeleton
  {
    namespace detail
    {

      /**
      * Write Skeleton.
      *
      * @param doc          The XML document that should be written.
      * @param skeleton     This argument holds the skeleton that should be written to the XML file.
      *
      * @return           If skeleton was written succesfully then the return value is true otherwise it is false.
      */
      template<typename skeleton_type>
      bool xml_write(TiXmlDocument & doc, skeleton_type const & skeleton)
      {
        typedef typename skeleton_type::math_types::coordsys_type    coordsys_type;
        typedef typename skeleton_type::bone_type                    bone_type;
        typedef typename bone_type::bone_traits                      bone_traits;
        typedef typename skeleton_type::const_bone_iterator          const_bone_iterator;

        TiXmlElement * skeleton_elem = new TiXmlElement( "skeleton" );
        doc.LinkEndChild(skeleton_elem);

        const_bone_iterator bone = skeleton.begin();
        const_bone_iterator end  = skeleton.end();
        for(;bone!=end;++bone)
        {
          TiXmlElement * bone_elem = new TiXmlElement( "bone" );

          if(bone->parent())
          {
            bone_elem->SetAttribute( "parent", bone->parent()->get_name() );
          }
          bone_elem->SetAttribute( "name", bone->get_name() );
          {
            coordsys_type bind_pose = bone_traits::convert( bone->bind_pose() );
            std::stringstream stream;
            stream.precision(15);
            stream << bind_pose;
            bone_elem->SetAttribute( "bindpose", stream.str() );
          }
          {
            coordsys_type bone_space = bone_traits::convert( bone->bone_space() );
            std::stringstream stream;
            stream.precision(15);
            stream << bone_space;
            bone_elem->SetAttribute( "bonespace", stream.str() );
          }
          skeleton_elem->LinkEndChild( bone_elem );
        }

        return true;
      }

    }// namespace detail

    /**
    * Write Skeleton.
    *
    * @param filename     The filename of the XML document that should be written.
    * @param skeleton     This argument holds the skeleton that should be written to the XML file.
    *
    * @return           If skeleton was written succesfully then the return value is true otherwise it is false.
    */
    template<typename skeleton_type>
    bool xml_write(std::string const & filename,skeleton_type const & skeleton)
    {
      // build document
      TiXmlDocument doc;

      TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
      doc.LinkEndChild(decl);

      if(!OpenTissue::skeleton::detail::xml_write( doc, skeleton ) )
        return false;

      // write the document
#ifdef TIXML_USE_STL
      doc.SaveFile(filename);
#else
      doc.SaveFile(filename.c_str());
#endif

      return true;
    }

  } // namespace skeleton
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_SKELETON_IO_SKELETON_XML_WRITE_H
#endif
