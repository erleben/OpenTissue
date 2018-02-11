#ifndef OPENTISSUE_KINEMATICS_INVERSE_IO_INVERSE_XML_READ_H
#define OPENTISSUE_KINEMATICS_INVERSE_IO_INVERSE_XML_READ_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/skeleton/io/skeleton_xml_read.h>

#include <TinyXML/tinyxml.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Read Solver from XML file.
      *
      * @param filename       The filename of the XML file that should be read.
      * @param solver         Upon return this argument holds the solver data that has been read from the XML file.
      * @param read_skeleton  Boolean flag indicating whether the skeleton data should be read as well.
      *
      *
      * @return              If the read operation was succesfully then the return value is true otherwise it is false.
      */
      template<typename solver_type>
      bool xml_read(std::string const & filename, solver_type & solver, bool const & read_skeleton = true)
      {
        typedef typename solver_type::skeleton_type    skeleton_type;
        typedef typename solver_type::math_types       math_types;
        typedef typename math_types::vector3_type      vector3_type;
        typedef typename math_types::real_type         real_type;
        typedef typename skeleton_type::bone_type      bone_type;
        typedef typename bone_type::bone_traits        bone_traits;

        if( !solver.skeleton() )
          throw std::invalid_argument( "solver has a skeleton null pointer" );

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

        if(read_skeleton)
        {
          if ( ! OpenTissue::skeleton::detail::xml_read( document_handle, *(solver.skeleton()) ) )
            return false;
        }

        solver.init( *(solver.skeleton()) );

        TiXmlElement * xml_joint = document_handle.FirstChild( "kinematics" ).FirstChild( "joint" ).Element();
        for( ; xml_joint; xml_joint=xml_joint->NextSiblingElement("joint") )
        {
          bone_type * bone = 0;

          if(xml_joint->Attribute("name"))
          {
            std::string name = xml_joint->Attribute("name");
            bone = solver.skeleton()->get_bone( name );
          }
          else
          {
            throw std::logic_error( "name attribute was missing in xml tag" );
          }

          if(!bone)
            throw std::logic_error( "could not find any bone of that name" );

          if(xml_joint->Attribute("type"))
          {
            std::string type = xml_joint->Attribute("type");

            if( type == "slider")
              bone->type() = bone_traits::slider_type;
            else if( type == "hinge")
              bone->type() = bone_traits::hinge_type;
            else if( type == "ball")
              bone->type() = bone_traits::ball_type;
          }

          if(xml_joint->Attribute("axis"))
          {
            vector3_type u;
            std::istringstream str_stream(xml_joint->Attribute("axis"));
            str_stream >> u;
            bone->u() = u ;
          }

          if(xml_joint->Attribute("min"))
          {
            std::istringstream str_stream(xml_joint->Attribute("min"));
            if(bone->type() == bone_traits::ball_type)
            {
              vector3_type min_values;
              str_stream >> min_values;
              bone->box_limits().min_limit(0) = min_values(0);
              bone->box_limits().min_limit(1) = min_values(1);
              bone->box_limits().min_limit(2) = min_values(2);
            }
            else
            {
              real_type min_value;
              str_stream >> min_value;
              bone->box_limits().min_limit(0) = min_value;
            }
          }

          if(xml_joint->Attribute("max"))
          {
            std::istringstream str_stream(xml_joint->Attribute("max"));
            if(bone->type() == bone_traits::ball_type)
            {
              vector3_type max_values;
              str_stream >> max_values;
              bone->box_limits().max_limit(0) = max_values(0);
              bone->box_limits().max_limit(1) = max_values(1);
              bone->box_limits().max_limit(2) = max_values(2);
            }
            else
            {
              real_type max_value;
              str_stream >> max_value;
              bone->box_limits().max_limit(0) = max_value;
            }
          }
        }

        TiXmlElement * xml_chain = document_handle.FirstChild( "kinematics" ).FirstChild( "chain" ).Element();
        for( ; xml_chain; xml_chain=xml_chain->NextSiblingElement("chain") )
        {
          bone_type const * root         = 0;
          bone_type const * end_effector = 0;

          if(xml_chain->Attribute("root"))
          {
            std::string name = xml_chain->Attribute("root");
            root = solver.skeleton()->get_bone( name );
          }
          else
          {
            throw std::logic_error( "root name attribute was missing in xml tag" );
          }
          if(xml_chain->Attribute("end"))
          {
            std::string name = xml_chain->Attribute("end");
            end_effector = solver.skeleton()->get_bone( name );
          }
          else
          {
            throw std::logic_error( "end effector name attribute was missing in xml tag" );
          }

          if(!root)
            throw std::logic_error( "could not find any bone of that name" );

          if(!end_effector)
            throw std::logic_error( "could not find any bone of that name" );

          typename solver_type::chain_type chain;
          chain.init( root, end_effector );

          if(xml_chain->Attribute("only_pos"))
          {
            bool value;
            std::istringstream str_stream(xml_chain->Attribute("only_pos"));
            str_stream >> value;
            chain.only_position() = value;
          }
          if(xml_chain->Attribute("weight"))
          {
            vector3_type weight;
            std::istringstream str_stream(xml_chain->Attribute("weight"));
            str_stream >> weight;
            chain.set_weight_p( weight(0) );
            chain.set_weight_x( weight(1) );
            chain.set_weight_y( weight(2) );
          }

          if(xml_chain->Attribute("p_global"))
          {
            vector3_type p_global;
            std::istringstream str_stream(xml_chain->Attribute("p_global"));
            str_stream >> p_global;
            chain.p_global() = p_global;
          }

          if(xml_chain->Attribute("p_local"))
          {
            vector3_type p_local;
            std::istringstream str_stream(xml_chain->Attribute("p_local"));
            str_stream >> p_local;
            chain.p_local() = p_local;
          }

          if(xml_chain->Attribute("x_global"))
          {
            vector3_type x_global;
            std::istringstream str_stream(xml_chain->Attribute("x_global"));
            str_stream >> x_global;
            chain.x_global() = x_global;
          }

          if(xml_chain->Attribute("x_local"))
          {
            vector3_type x_local;
            std::istringstream str_stream(xml_chain->Attribute("x_local"));
            str_stream >> x_local;
            chain.x_local() = x_local;
          }

          if(xml_chain->Attribute("y_global"))
          {
            vector3_type y_global;
            std::istringstream str_stream(xml_chain->Attribute("y_global"));
            str_stream >> y_global;
            chain.y_global() = y_global;
          }

          if(xml_chain->Attribute("y_local"))
          {
            vector3_type y_local;
            std::istringstream str_stream(xml_chain->Attribute("y_local"));
            str_stream >> y_local;
            chain.y_local() = y_local;
          }

          solver.add_chain( chain );
        }

        xml_document.Clear();
        return true;
      }

    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_IO_INVERSE_XML_READ_H
#endif
