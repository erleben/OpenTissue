#ifndef OPENTISSUE_KINEMATICS_INVERSE_IO_INVERSE_XML_WRITE_H
#define OPENTISSUE_KINEMATICS_INVERSE_IO_INVERSE_XML_WRITE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/kinematics/skeleton/io/skeleton_xml_write.h>

#include <TinyXML/tinyxml.h>

#include <stdexcept>

namespace OpenTissue
{
  namespace kinematics
  {
    namespace inverse
    {

      /**
      * Write Solver to XML file.
      *
      * @param filename        The filename of the XML file that should be written.
      * @param solver          This argument holds the solver data that should be written to the XML file.
      * @param write_skeleton  Boolean flag indicating whether skeleton info should be written as well.
      *
      *
      * @return              If the write operation was succesfully then the return value is true otherwise it is false.
      */
      template<typename solver_type>
      bool xml_write(std::string const & filename, solver_type const & solver, bool const & write_skeleton = true)
      {
        typedef typename solver_type::skeleton_type          skeleton_type;
        typedef typename solver_type::math_types             math_types;
        typedef typename math_types::vector3_type            vector3_type;
        typedef typename math_types::real_type               real_type;
        typedef typename skeleton_type::bone_type            bone_type;
        typedef typename bone_type::bone_traits              bone_traits;
        typedef typename skeleton_type::const_bone_iterator  const_bone_iterator;

        // build document
        TiXmlDocument doc;

        TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
        TiXmlElement * ik_elem = new TiXmlElement( "kinematics" );

        doc.LinkEndChild(decl);
        doc.LinkEndChild(ik_elem);

        if( !solver.skeleton() )
          throw std::invalid_argument( "solver has a skeleton null pointer" );

        if(write_skeleton)
        {
          if(! OpenTissue::skeleton::detail::xml_write( doc, *(solver.skeleton()) ) )
            throw std::logic_error( "could not write skeleton to xml file" );
        }

        const_bone_iterator bone  = solver.skeleton()->begin();
        const_bone_iterator b_end = solver.skeleton()->end();

        for(;bone!=b_end;++bone)
        {
          TiXmlElement * joint_elem = new TiXmlElement( "joint" );

          joint_elem->SetAttribute( "name", bone->get_name() );

          if(bone->type()==bone_traits::slider_type)
          {
            joint_elem->SetAttribute( "type", "slider" );
            {
              vector3_type u = bone->u();
              std::stringstream u_stream;
              u_stream.precision(15);
              u_stream << u;
              joint_elem->SetAttribute( "axis", u_stream.str() );
            }
            {
              std::stringstream min_stream;
              min_stream.precision(15);
              min_stream << bone->box_limits().min_limit(0);
              joint_elem->SetAttribute( "min", min_stream.str() );
            }
            {
              std::stringstream max_stream;
              max_stream.precision(15);
              max_stream << bone->box_limits().max_limit(0);
              joint_elem->SetAttribute( "max", max_stream.str() );
            }
          }
          if(bone->type()==bone_traits::hinge_type)
          {
            joint_elem->SetAttribute( "type", "hinge" );
            {
              vector3_type u = bone->u();
              std::stringstream u_stream;
              u_stream.precision(15);
              u_stream << u;
              joint_elem->SetAttribute( "axis", u_stream.str() );
            }
            {
              std::stringstream min_stream;
              min_stream.precision(15);
              min_stream << bone->box_limits().min_limit(0);
              joint_elem->SetAttribute( "min", min_stream.str() );
            }
            {
              std::stringstream max_stream;
              max_stream.precision(15);
              max_stream << bone->box_limits().max_limit(0);
              joint_elem->SetAttribute( "max", max_stream.str() );
            }
          }
          if(bone->type()==bone_traits::ball_type)
          {
            joint_elem->SetAttribute( "type", "ball" );

            {
              vector3_type min_vector;
              min_vector(0) = bone->box_limits().min_limit(0);
              min_vector(1) = bone->box_limits().min_limit(1);
              min_vector(2) = bone->box_limits().min_limit(2);
              std::stringstream min_stream;
              min_stream.precision(15);
              min_stream << min_vector;
              joint_elem->SetAttribute( "min", min_stream.str() );
            }

            {
              vector3_type max_vector;
              max_vector(0) = bone->box_limits().max_limit(0);
              max_vector(1) = bone->box_limits().max_limit(1);
              max_vector(2) = bone->box_limits().max_limit(2);
              std::stringstream max_stream;
              max_stream.precision(15);
              max_stream << max_vector;
              joint_elem->SetAttribute( "max", max_stream.str() );
            }
          }

          ik_elem->LinkEndChild( joint_elem );
        }

        typename solver_type::const_chain_iterator  chain = solver.chain_begin();
        typename solver_type::const_chain_iterator  c_end = solver.chain_end();
        for(;chain!=c_end;++chain)
        {
          TiXmlElement * chain_elem = new TiXmlElement( "chain" );

          std::string root_name = chain->get_root()->get_name();
          std::string end_effector_name = chain->get_end_effector()->get_name();
          chain_elem->SetAttribute( "root", root_name );
          chain_elem->SetAttribute( "end", end_effector_name );

          chain_elem->SetAttribute( "only_pos", chain->only_position() );
          {
            vector3_type weight;
            weight(0) = chain->weight_p();
            weight(1) = chain->weight_x();
            weight(2) = chain->weight_y();
            std::stringstream weight_stream;
            weight_stream.precision(15);
            weight_stream << weight;
            chain_elem->SetAttribute( "weight", weight_stream.str() );
          }
          {
            std::stringstream stream;
            stream.precision(15);
            stream << chain->p_global();
            chain_elem->SetAttribute( "p_global", stream.str() );
          }
          {
            std::stringstream stream;
            stream.precision(15);
            stream << chain->p_local();
            chain_elem->SetAttribute( "p_local", stream.str() );
          }
          {
            std::stringstream stream;
            stream.precision(15);
            stream << chain->x_global();
            chain_elem->SetAttribute( "x_global", stream.str() );
          }
          {
            std::stringstream stream;
            stream.precision(15);
            stream << chain->x_local();
            chain_elem->SetAttribute( "x_local", stream.str() );
          }
          {
            std::stringstream stream;
            stream.precision(15);
            stream << chain->y_global();
            chain_elem->SetAttribute( "y_global", stream.str() );
          }
          {
            std::stringstream stream;
            stream.precision(15);
            stream << chain->y_local();
            chain_elem->SetAttribute( "y_local", stream.str() );
          }
          ik_elem->LinkEndChild( chain_elem );
        }

        // write the document
#ifdef TIXML_USE_STL
        doc.SaveFile(filename);
#else
        doc.SaveFile(filename.c_str());
#endif

        return true;
      }


    } // namespace inverse
  } // namespace kinematics
} // namespace OpenTissue

//OPENTISSUE_KINEMATICS_INVERSE_IO_INVERSE_XML_WRITE_H
#endif
