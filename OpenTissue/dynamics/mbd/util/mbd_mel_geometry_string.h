#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_GEOMETRY_STRING_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_GEOMETRY_STRING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include<string>
#include<sstream>

namespace OpenTissue
{
  namespace mbd
  {
    namespace mel
    {

      namespace detail
      {

        template<typename mesh_type,typename index_type>
        void polygon_geometry(std::stringstream & stream,mesh_type * mesh, index_type idx)
        {
          typedef typename mesh_type::face_iterator   face_iterator;
          typedef typename mesh_type::vertex_type     vertex_type;
          typedef typename mesh_type::halfedge_type   halfedge_type;


          int unused = static_cast<int>(mesh->size_vertices() +1);

          mesh::clear_vertex_tags(*mesh, unused);
          int cnt = 0;

          bool firstFace = true;
          for(face_iterator face = mesh->face_begin();face!=mesh->face_end();++face)
          {
            assert( valency(*face)==3);

            typename mesh_type::face_halfedge_circulator h(*face);
            halfedge_type * edge0 = &(*h);++h;
            halfedge_type * edge1 = &(*h);++h;
            halfedge_type * edge2 = &(*h);

            assert(edge0->get_face_handle() == edge1->get_face_handle());
            assert(edge0->get_face_handle() == edge2->get_face_handle());

            vertex_type * v0 = &(*edge0->get_origin_iterator());
            vertex_type * v1 = &(*edge1->get_origin_iterator());
            vertex_type * v2 = &(*edge2->get_origin_iterator());

            if(v0->m_tag==unused && v1->m_tag==unused && v2->m_tag==unused)
            {
              v0->m_tag = cnt++;
              v1->m_tag = cnt++;
              v2->m_tag = cnt++;
              if(firstFace)
              {
                stream << "polyCreateFacet -ch off"
                  << " -p "     << v0->m_coord(0) << " " << v0->m_coord(1) << " "<< v0->m_coord(2)
                  << " -p "     << v1->m_coord(0) << " " << v1->m_coord(1) << " "<< v1->m_coord(2)
                  << " -p "     << v2->m_coord(0) << " " << v2->m_coord(1) << " "<< v2->m_coord(2)
                  << " -n body" << idx
                  << ";"        << std::endl;
                firstFace = false;
              }
              else
              {
                stream << "polyAppendVertex -ch off"
                  << " -p "     << v0->m_coord(0) << " " << v0->m_coord(1) << " "<< v0->m_coord(2)
                  << " -p "     << v1->m_coord(0) << " " << v1->m_coord(1) << " "<< v1->m_coord(2)
                  << " -p "     << v2->m_coord(0) << " " << v2->m_coord(1) << " "<< v2->m_coord(2)
                  << ";"        << std::endl;
              }
            }

            /////////////////////////////////////////////////////////////////////////////////////////7
            else if(v0->m_tag!=unused && v1->m_tag==unused && v2->m_tag==unused)
            {
              v1->m_tag = cnt++;
              v2->m_tag = cnt++;
              stream << "polyAppendVertex -ch off"
                << " -v "     << v0->m_tag
                << " -p "     << v1->m_coord(0) << " " << v1->m_coord(1) << " "<< v1->m_coord(2)
                << " -p "     << v2->m_coord(0) << " " << v2->m_coord(1) << " "<< v2->m_coord(2)
                << ";"        << std::endl;
            }
            /////////////////////////////////////////////////////////////////////////////////////////7
            else if(v0->m_tag==unused && v1->m_tag!=unused && v2->m_tag==unused)
            {
              v0->m_tag = cnt++;
              v2->m_tag = cnt++;
              stream << "polyAppendVertex -ch off"
                << " -p "     << v0->m_coord(0) << " " << v0->m_coord(1) << " "<< v0->m_coord(2)
                << " -v "     << v1->m_tag
                << " -p "     << v2->m_coord(0) << " " << v2->m_coord(1) << " "<< v2->m_coord(2)
                << ";"        << std::endl;
            }
            /////////////////////////////////////////////////////////////////////////////////////////7
            else if(v0->m_tag==unused && v1->m_tag==unused && v2->m_tag!=unused)
            {
              v0->m_tag = cnt++;
              v1->m_tag = cnt++;
              stream << "polyAppendVertex -ch off"
                << " -p "     << v0->m_coord(0) << " " << v0->m_coord(1) << " "<< v0->m_coord(2)
                << " -p "     << v1->m_coord(0) << " " << v1->m_coord(1) << " "<< v1->m_coord(2)
                << " -v "     << v2->m_tag
                << ";"        << std::endl;
            }
            /////////////////////////////////////////////////////////////////////////////////////////7
            else if(v0->m_tag!=unused && v1->m_tag!=unused && v2->m_tag==unused)
            {
              v2->m_tag = cnt++;
              stream << "polyAppendVertex -ch off"
                << " -v "     << v0->m_tag
                << " -v "     << v1->m_tag
                << " -p "     << v2->m_coord(0) << " " << v2->m_coord(1) << " "<< v2->m_coord(2)
                << ";"        << std::endl;
            }
            else if(v0->m_tag==unused && v1->m_tag!=unused && v2->m_tag!=unused)
            {
              v0->m_tag = cnt++;
              stream << "polyAppendVertex -ch off"
                << " -p "     << v0->m_coord(0) << " " << v0->m_coord(1) << " "<< v0->m_coord(2)
                << " -v "     << v1->m_tag
                << " -v "     << v2->m_tag
                << ";"        << std::endl;
            }
            else if(v0->m_tag!=unused && v1->m_tag==unused && v2->m_tag!=unused)
            {
              v1->m_tag = cnt++;
              stream << "polyAppendVertex -ch off"
                << " -v "     << v0->m_tag
                << " -p "     << v1->m_coord(0) << " " << v1->m_coord(1) << " "<< v1->m_coord(2)
                << " -v "     << v2->m_tag
                << ";"        << std::endl;
            }
            else if(v0->m_tag!=unused && v1->m_tag!=unused && v2->m_tag!=unused)
            {
              stream << "polyAppendVertex -ch off"
                << " -v "     << v0->m_tag
                << " -v "     << v1->m_tag
                << " -v "     << v2->m_tag
                << ";"        << std::endl;
            }
          }
          stream << std::endl;
          stream << std::endl;
        }



        template<typename sphere_type, typename index_type>
        void sphere_geometry(std::stringstream & stream,sphere_type const & sphere,index_type idx)
        {
          stream << "polySphere -r " << sphere.radius() << " -ch 0 -n body" << idx << ";" << std::endl;
        }

        template<typename box_type, typename index_type>
        void box_geometry(std::stringstream & stream,box_type const & box,index_type idx)
        {
          stream << "  polyCube "
            << "-w " << 2.*box.ext()(0) << " "
            << "-h " << 2.*box.ext()(1) << " "
            << "-d " << 2.*box.ext()(2) << " "
            << "-ch 0 -n body" << idx << ";" <<std::endl;
        }

        template<typename plane_type, typename index_type>
        void plane_geometry(std::stringstream & stream, plane_type const & /*plane*/, index_type idx)
        {
          stream << "polyPlane "
            << "-w " << 1000 << " "
            << "-h " << 1000 << " "
            << "-ch 0 -n body" << idx << ";" << std::endl;
        }

      } // namespace detail

      /**
      * MEL geometry String Tool.
      * Example usage:
      *
      * std::cout << mbd::mel::geometry_string(configuration.body_begin(),configuration.body_end(),simulator.get_time())  << std::endl;
      *
      */
      template< typename indirect_body_iterator>
      std::string geometry_string(indirect_body_iterator begin, indirect_body_iterator end)
      {
        typedef typename indirect_body_iterator::value_type     body_type;

        typedef typename body_type::math_policy           math_policy;

        typedef OpenTissue::geometry::Sphere<math_policy>       sphere_type;
        typedef OpenTissue::geometry::Plane<math_policy>        plane_type;
        typedef OpenTissue::geometry::OBB<math_policy>          box_type;
        typedef OpenTissue::polymesh::PolyMesh<math_policy>     mesh_type;
        typedef OpenTissue::grid::Grid<float,math_policy>       grid_type;
        typedef OpenTissue::sdf::Geometry<mesh_type,grid_type>  sdf_geometry_type;


        std::stringstream stream;
        for(indirect_body_iterator body=begin;body!=end;++body)
        {

          // 2008-06-13 kenny: this is down-right ugly! Stuf like this should
          // not be part of a multibody simulator! And if functionality like
          // this should be supported it would proberly be better to do it with
          // virtual interfaces or using a dispatcher.
          if(body->get_geometry()->class_id()==sdf_geometry_type::id() )
          {
            sdf_geometry_type * sdf = static_cast<sdf_geometry_type*>(body->get_geometry() );
            detail::polygon_geometry(stream,&(sdf->m_mesh),body->get_index());
          }
          else if(body->get_geometry()->class_id()==box_type::id() )
          {
            box_type* box = static_cast<box_type *>(body->get_geometry() );
            detail::box_geometry(stream,*box,body->get_index());
          }
          else if(body->get_geometry()->class_id()==sphere_type::id() )
          {
            sphere_type* sphere = static_cast<sphere_type *>(body->get_geometry() );
            detail::sphere_geometry(stream,*sphere,body->get_index());
          }
          else if(body->get_geometry()->class_id()==plane_type::id() )
          {
            plane_type* plane = static_cast<plane_type *>(body->get_geometry() );
            detail::plane_geometry(stream,*plane,body->get_index());
          }
          else
          {
            assert(!"mbd::mel::geometry_string(): geometry type not handled, sorry");
          }
        }
        return stream.str();
      }

    } // namespace mel
  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_MBD_MEL_GEOMETRY_STRING_H
#endif
