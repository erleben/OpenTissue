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
#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>
#include <OpenTissue/collision/bvh/bvh.h>

#include <OpenTissue/utility/utility_timer.h>

#include "curvature_mesh_volume_refitter.h"
#include "curvature_mesh_construction_policy.h"
#include "curvature_mesh_bv_traits.h"
#include "curvature_mesh_collision_policy.h"


class Application : public OpenTissue::glut::PerspectiveViewApplication
{
private:

  typedef double                                               real_type;
  typedef OpenTissue::math::BasicMathTypes<real_type, size_t>  math_types;
  typedef math_types::vector3_type                             vector3_type;

  typedef OpenTissue::geometry::AABB<math_types>               aabb_type;
  typedef OpenTissue::polymesh::PolyMesh<>                     mesh_type;
  typedef mesh_type::face_type                                 face_type;

  typedef OpenTissue::bvh::BoundingVolumeHierarchy<aabb_type,face_type*,CurvatureMeshBVTraits>                           bvh_type;
  typedef OpenTissue::bvh::BVHGraph<bvh_type>                                                             graph_type;
  typedef OpenTissue::bvh::BottomUpConstructor<bvh_type,CurvatureMeshConstructionPolicy<bvh_type> >    constructor_type;
  typedef OpenTissue::bvh::Mesh2BVHGraph<graph_type>                                                      converter_type;
  typedef OpenTissue::bvh::BottomUpRefitter< CurvatureMeshVolumeRefitter<bvh_type> >                   refitter_type;

  bvh_type         m_bvh;                 ///< The BVH data structure.
  graph_type       m_graph;               ///< Graph data structure used as initial input for the bottom-up constructor.
  converter_type   m_converter;           ///< A volume to graph conversion utility. Takes a set of initial volumes and creates a graph.
  constructor_type m_constructor;         ///< The bottom up constructor.
  refitter_type    m_refitter;            ///< Bottom up refitting of BVH.
  mesh_type        m_mesh;                ///< The original mesh, from which the BVH is builded.

private:

  typedef OpenTissue::bvh::SelfCollisionQuery<CurvatureMeshCollisionPolicy<bvh_type> >   collision_query_type;
  typedef collision_query_type::result_type                                            result_type;
  typedef collision_query_type::results_container                                      results_container;
  collision_query_type    m_collision_query;            ///< Collision query.
  results_container       m_results;                    ///< Results from collision query.

private:

  unsigned int m_depth;   //--- Controls to what depth the BVH shoud be drawn
  unsigned int m_height;  //--- Controls from what height the BVH shoud be drawn

public:

  Application()
    : m_depth(0)
    , m_height(1)
  { }

public:

  char const * do_get_title() const { return "Curvature Mesh BVH Demo Application"; }

  void do_display()
  {
    bvh_type::bv_ptr_container nodes;

    OpenTissue::gl::ColorPicker(0,0,.7);
    OpenTissue::gl::DrawMesh(m_mesh,GL_LINE_LOOP);

    OpenTissue::bvh::get_nodes_at_height(m_bvh,1,nodes);
    OpenTissue::gl::ColorPicker(0,.7,0);
    bvh_type::bv_iterator node = nodes.begin();
    bvh_type::bv_iterator end  = nodes.end();
    for (;node!= end; ++node )
      OpenTissue::gl::DrawAABB(node->volume() , false );

    OpenTissue::bvh::get_nodes_at_depth(m_bvh,m_depth,nodes);
    OpenTissue::gl::ColorPicker(1,0,0);
    node = nodes.begin();
    end  = nodes.end();
    for (;node!= end;++node )
      OpenTissue::gl::DrawAABB( node->volume(), true );
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
        << "height = " 
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
          << "height = " 
          << m_height
          << std::endl;
      }
      break;
    case 'b':
      {
        std::string data_path  = opentissue_path;

        std::string meshfile = data_path + "/demos/data/obj/propella.obj";
        OpenTissue::mesh::obj_read( meshfile, m_mesh );

        OpenTissue::utility::Timer<real_type> watch;
        watch.start();
        m_converter.run( m_mesh, m_graph );
        watch.stop();
        std::cout << "BVH graph conversion took " << watch() << " secs." << std::endl;

        watch.start();
        m_constructor.run( m_graph, m_bvh );
        watch.stop();
        std::cout << "BVH construction took " << watch() << " secs." << std::endl;

        bvh_type::bv_ptr_container nodes;
        OpenTissue::bvh::get_all_nodes(m_bvh,nodes);
        unsigned int adj_total = 0;
        unsigned int adj_min = 0xFFFF;
        unsigned int adj_max = 0;
        unsigned int branch_max = 0;
        unsigned int branch_total = 0;
        bvh_type::bv_iterator bv  = nodes.begin();
        bvh_type::bv_iterator end = nodes.end();
        for(;bv!=end;++bv)
        {
          adj_total += static_cast<unsigned int>( bv->m_adjacency.size() );
          if(bv->m_adjacency.size()>adj_max)
            adj_max = static_cast<unsigned int>( bv->m_adjacency.size() );
          if(bv->m_adjacency.size()!=0 && bv->m_adjacency.size()<adj_min)
            adj_min = static_cast<unsigned int>( bv->m_adjacency.size() );
          branch_total += bv->children();
          if(bv->children()>branch_max)
            branch_max = bv->children();
        }
        std::cout << "Average adjacency list contains " << ((1.0*adj_total)/nodes.size()) << " vertices" << std::endl;
        std::cout << "Minimum adjacency list contains " << adj_min << " vertices" << std::endl;
        std::cout << "Maximum adjacency list contains " << adj_max << " vertices" << std::endl;
        std::cout << "Maximum brancing factor " << branch_max << " branches" << std::endl;
        std::cout << "Average branching factor " << ((1.0*branch_total)/nodes.size()) << " branches" << std::endl;
      }
      break;
    case 'd':
      {
        //--- Apply some deformation...
        OpenTissue::mesh::spherical_bend(m_mesh,vector3_type(0.0,1.0,0.0),2.0);

        bvh_type::bv_ptr_container leaves;
        OpenTissue::bvh::get_leaf_nodes(m_bvh,leaves);
        m_refitter.run(leaves);
        m_results.clear();

        OpenTissue::utility::Timer<real_type> watch;

        watch.start();
        m_collision_query.run(m_bvh,m_results);
        watch.stop();
        std::cout << "Self collision query took " 
                  << watch() 
                  << " secs. found " 
                  << m_results.size()
                  << " collision" 
                  << std::endl;
      }
      break;
    case 'r':
      {
        bvh_type::bv_ptr_container leaves;
        OpenTissue::bvh::get_leaf_nodes(m_bvh,leaves);
        OpenTissue::utility::Timer<real_type> watch;

        watch.start();
        m_refitter.run(leaves);
        watch.stop();
        std::cout << "BVH refit took " << watch() << " secs." << std::endl;
      }
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "Build Curvature Mesh BVH  [b]", 'b' );
    glutAddMenuEntry( "Deform Curvature Mesh BVH [d]", 'd' );
    glutAddMenuEntry( "Refit Curvature Mesh BVH  [r]", 'r' );
    glutAddMenuEntry( "Decrease BVH height       [-]", '-' );
    glutAddMenuEntry( "Increase BVH height       [+]", '+' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "curvature mesh", controls );
  }

  void do_init()
  {
    this->camera().move(95);
  }

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
