//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/utility/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN

#include <OpenTissue/core/math/math_basic_types.h>

#include <OpenTissue/collision/bvh/bvh.h>
#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/core/containers/t4mesh/io/t4mesh_xml_read.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_delaunay_tetrahedralization.h>
#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/core/geometry/geometry_tetrahedron.h>

#include "t4mesh_volume_refitter.h"
#include "t4mesh_construction_policy.h"
#include "t4mesh_collision_policy.h"

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double, size_t>                       math_types;
  typedef math_types::vector3_type                                               vector3_type;
  typedef OpenTissue::geometry::AABB<math_types>                                 aabb_type;
  typedef OpenTissue::t4mesh::T4Mesh<math_types>                                 mesh_type;
  typedef mesh_type::tetrahedron_type                                            tetrahedron_type;
  typedef OpenTissue::bvh::BoundingVolumeHierarchy< aabb_type, tetrahedron_type* >                   bvh_type;
  typedef OpenTissue::bvh::BVHGraph<bvh_type>                                         graph_type;
  typedef OpenTissue::bvh::BottomUpConstructor<bvh_type,T4MeshConstructionPolicy<bvh_type> > constructor_type;
  typedef OpenTissue::bvh::T4Mesh2BVHGraph<mesh_type,graph_type>                      converter_type;
  typedef OpenTissue::bvh::BottomUpRefitter< T4MeshVolumeRefitter<bvh_type> >      refitter_type;

  bvh_type         m_bvh;                 ///< The BVH data structure.
  graph_type       m_graph;               ///< Graph data structure used as initial input for the bottom-up constructor.
  converter_type   m_converter;           ///< A volume to graph conversion utility. Takes a set of initial volumes and creates a graph.
  constructor_type m_constructor;         ///< The bottom up constructor.
  refitter_type    m_refitter;            ///< Bottom up refitting of BVH.
  mesh_type        m_mesh;                ///< The original mesh, from which the BVH is builded.

  typedef std::vector<vector3_type> coord_container;
  coord_container m_coords;               ///< Coordinates of nodes in mesh.


  typedef OpenTissue::bvh::WorldCollisionQuery<T4MeshCollisionPolicy<bvh_type> > collision_query_type;
  typedef collision_query_type::result_type                                    result_type;
  typedef collision_query_type::results_container                              results_container;

  coord_container m_coords_A;               ///< Coordinates of nodes in mesh A.
  coord_container m_coords_B;               ///< Coordinates of nodes in mesh B.
  mesh_type        m_mesh_A;                ///< The original mesh of object A.
  mesh_type        m_mesh_B;                ///< The original mesh of object B.
  bvh_type         m_bvh_A;                 ///< The BVH data structure of object A.
  bvh_type         m_bvh_B;                 ///< The BVH data structure of object B.

  collision_query_type m_collision_query;         ///< Collision query.
  results_container m_results;                    ///< Results from collision query.

  unsigned int m_depth;   //--- Controls to what depth the BVH shoud be drawn
  unsigned int m_height;  //--- Controls from what height the BVH shoud be drawn

public:

  Application()
    : m_depth(0)
    , m_height(1) 
  {
  }

public:

  char const * do_get_title() const { return "T4Mesh BVH Demo Application"; }

  void do_display()
  {
    OpenTissue::gl::ColorPicker(.3,.3,0);

    bvh_type::bv_ptr_container nodes;

    OpenTissue::bvh::get_nodes_at_height(m_bvh,1,nodes);
    OpenTissue::gl::ColorPicker(0,.7,0);
    bvh_type::bv_iterator node = nodes.begin();
    bvh_type::bv_iterator end  = nodes.end();
    for (;node!= end;++node )
      OpenTissue::gl::DrawAABB(node->volume(), false );

    OpenTissue::bvh::get_nodes_at_closest_height(m_bvh,m_height,nodes);
    OpenTissue::gl::ColorPicker(1,0,0);
    node = nodes.begin();
    end  = nodes.end();
    for (;node!= end;++node )
      OpenTissue::gl::DrawAABB( node->volume() , true );

    OpenTissue::gl::ColorPicker(.7,0,0);
    OpenTissue::gl::DrawPointsT4Mesh(m_coords_A,m_mesh_A,0.95,true);
    OpenTissue::gl::ColorPicker(0,.7,0);
    OpenTissue::gl::DrawPointsT4Mesh(m_coords_B,m_mesh_B,0.95,true);
    for(unsigned int i=0;i<m_results.size();++i)
    {
      unsigned int idx_i_A = m_mesh_A.tetrahedron(m_results[i].m_idx_A)->local2global(0);
      unsigned int idx_j_A = m_mesh_A.tetrahedron(m_results[i].m_idx_A)->local2global(1);
      unsigned int idx_k_A = m_mesh_A.tetrahedron(m_results[i].m_idx_A)->local2global(2);
      unsigned int idx_m_A = m_mesh_A.tetrahedron(m_results[i].m_idx_A)->local2global(3);

      OpenTissue::geometry::Tetrahedron<math_types> TA;
      TA.set( m_coords_A[idx_i_A], m_coords_A[idx_j_A], m_coords_A[idx_k_A], m_coords_A[idx_m_A] );
      OpenTissue::gl::ColorPicker(.7,0,0);
      OpenTissue::gl::DrawTetrahedron( TA, false);
      unsigned int idx_i_B = m_mesh_B.tetrahedron(m_results[i].m_idx_B)->local2global(0);
      unsigned int idx_j_B = m_mesh_B.tetrahedron(m_results[i].m_idx_B)->local2global(1);
      unsigned int idx_k_B = m_mesh_B.tetrahedron(m_results[i].m_idx_B)->local2global(2);
      unsigned int idx_m_B = m_mesh_B.tetrahedron(m_results[i].m_idx_B)->local2global(3);
      OpenTissue::geometry::Tetrahedron<math_types> TB;
      TB.set( m_coords_B[idx_i_B], m_coords_B[idx_j_B], m_coords_B[idx_k_B], m_coords_B[idx_m_B] );
      OpenTissue::gl::ColorPicker(0,.7,0);
      OpenTissue::gl::DrawTetrahedron( TB, false);
    }
  }

  void do_action(unsigned char choice)
  {
    switch ( choice )
    {
    case '+':
      m_depth++;
      m_height++;
      std::cout << "depth = "
        << m_depth
        << " height = " 
        << m_height
        << std::endl;
      break;
    case '-':
      {
        if ( m_depth > 0 )
          --m_depth;
        m_height--;
        if ( m_height < 1 )
          m_height = 1;
        std::cout << "depth = "
          << m_depth
          << " height = " 
          << m_height
          << std::endl;
      }
      break;
    case 'b':
      {
        std::string data_path = opentissue_path;
        OpenTissue::t4mesh::xml_read(data_path + "/demos/data/xml/blocky.xml", m_mesh, m_coords);
        m_converter.run( m_mesh, m_graph );
        m_constructor.set_coords(m_coords);
        m_constructor.run( m_graph, m_bvh );
      }
      break;
    case 'c':
      {
        m_coords_A.resize(10);
        for(unsigned int i=0;i<10;++i)
          random(m_coords_A[i],-1.5,0.5);

        OpenTissue::t4mesh::delaunay_tetrahedralization(m_coords_A,m_mesh_A);
        m_converter.run( m_mesh_A, m_graph );
        m_constructor.set_coords(m_coords_A);
        m_constructor.run( m_graph, m_bvh_A );

        m_coords_B.resize(10);
        for(unsigned int i=0;i<10;++i)
          random(m_coords_B[i],-.5,1.5);

        OpenTissue::t4mesh::delaunay_tetrahedralization(m_coords_B,m_mesh_B);
        m_converter.run( m_mesh_B, m_graph );
        m_constructor.set_coords(m_coords_B);
        m_constructor.run( m_graph, m_bvh_B );

        m_results.clear();
        m_collision_query.set_coords_A(m_coords_A);
        m_collision_query.set_coords_B(m_coords_B);
        m_collision_query.run(m_bvh_A,m_bvh_B,m_results);
      }
      break;
    case 'r':
      {
        bvh_type::bv_ptr_container leaves;
        OpenTissue::bvh::get_leaf_nodes(m_bvh,leaves);
        vector3_type r;
        coord_container deformation(m_coords.size());
        for(unsigned int i =0;i<m_coords.size();++i)
        {
          random(r,-.1,.1);
          deformation[i] = m_coords[i] + r;
        }
        m_refitter.set_coords(deformation);
        m_refitter.run(leaves);
      }
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "Build t4mesh BVH    [b]", 'b' );
    glutAddMenuEntry( "Refit t4mesh BVH    [r]", 'r' );
    glutAddMenuEntry( "Collision query of two t4mesh BVH  [c]", 'c' );
    glutAddMenuEntry( "Decrease BVH height [-]", '-' );
    glutAddMenuEntry( "Increase BVH height [+]", '+' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "T4Mesh BVH", controls );
  }

  void do_init(){}

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
