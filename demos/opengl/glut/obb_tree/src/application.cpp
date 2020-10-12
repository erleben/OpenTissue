//
// OpenTissue Template Library Demo
// - A specific demonstration of the flexibility of OTTL.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL and OTTL Demos are licensed under zlib.
//
#include <OpenTissue/configuration.h>

#define DEFINE_GLUT_MAIN
#include <OpenTissue/graphics/glut/glut_perspective_view_application.h>
#undef DEFINE_GLUT_MAIN

#include <OpenTissue/collision/obb_tree/obb_tree.h>
#include <OpenTissue/collision/collision_obb_tree_obb_tree.h>

#include <OpenTissue/core/containers/mesh/common/io/mesh_obj_read.h>
#include <OpenTissue/utility/utility_timer.h>

#include <OpenTissue/collision/bvh/bvh_get_nodes_at_height.h>
#include <OpenTissue/collision/bvh/bvh_get_nodes_at_depth.h>



class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
private:

  typedef OpenTissue::obb_tree::OBBTreeTypes<>  types;
  typedef types::bvh_type                       tree_type;
  typedef types::mesh_type             mesh_type;
  typedef types::coordsys_type                  coordsys_type;
  typedef types::vector3_type                   vector3_type;
  typedef types::quaternion_type                quaternion_type;
  typedef types::real_type                      real_type;

  coordsys_type         m_Awcs;
  coordsys_type         m_Bwcs;
  tree_type             m_treeA;
  tree_type             m_treeB;
  mesh_type             m_meshA;
  mesh_type             m_meshB;

  unsigned int m_depth;   ///< Controls to what depth the BVH shoud be drawn
  unsigned int m_height;  ///< Controls from what height the BVH shoud be drawn

public:

  Application()
    : m_depth(0)
    , m_height(0)
  { }

public:

  char const * do_get_title() const { return "OBB Tree Demo Application"; }

  void do_display()
  {
    tree_type::bv_ptr_container nodes;

    glPushMatrix();
    OpenTissue::gl::Transform(m_Awcs);

    OpenTissue::gl::ColorPicker(0.7,0,0);
    OpenTissue::gl::DrawMesh(m_meshA,GL_LINE_LOOP);

    OpenTissue::bvh::get_nodes_at_height(m_treeA,1,nodes);
    OpenTissue::gl::ColorPicker(0,.7,0);
    tree_type::bv_iterator node = nodes.begin();
    tree_type::bv_iterator end = nodes.end();
    for (;node!=end;++node )
      OpenTissue::gl::DrawOBB( node->volume(), false );

    OpenTissue::bvh::get_nodes_at_depth(m_treeA,m_depth,nodes);
    OpenTissue::gl::ColorPicker(1,0,0);
    node = nodes.begin();
    end = nodes.end();
    for (;node!=end;++node )
      OpenTissue::gl::DrawOBB( node->volume() , true );

    glPopMatrix();
    glPushMatrix();
    OpenTissue::gl::Transform(m_Bwcs);

    OpenTissue::gl::ColorPicker(0,0,.7);
    OpenTissue::gl::DrawMesh(m_meshB,GL_LINE_LOOP);

    OpenTissue::bvh::get_nodes_at_height(m_treeB,1,nodes);
    OpenTissue::gl::ColorPicker(0,.7,0);
    node = nodes.begin();
    end = nodes.end();
    for (;node!=end;++node )
      OpenTissue::gl::DrawOBB( node->volume() , false );

    OpenTissue::bvh::get_nodes_at_depth(m_treeB,m_depth,nodes);
    OpenTissue::gl::ColorPicker(1,0,0);
    node = nodes.begin();
    end = nodes.end();
    for (;node!=end;++node )
      OpenTissue::gl::DrawOBB( node->volume() , true );
    glPopMatrix();
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
        OpenTissue::mesh::obj_read( data_path + "/demos/data/obj/torus.obj", m_meshA );
        OpenTissue::mesh::obj_read( data_path + "/demos/data/obj/box.obj", m_meshB );

        OpenTissue::utility::Timer<real_type> watch;
        watch.start();
        OpenTissue::obb_tree::init<types>(m_meshA,m_treeA);
        //OpenTissue::obb_tree::init<types>(m_meshA,m_treeA,4);   // In case one wants a tree with cardinality 4.
        watch.stop();
        std::cout << "OBB Tree construction took " << watch() << " secs." << std::endl;

        watch.start();
        OpenTissue::obb_tree::init<types>(m_meshB,m_treeB);
        watch.stop();
        std::cout << "OBB Tree construction took " << watch() << " secs." << std::endl;

        m_Awcs = coordsys_type( vector3_type(2,0,0),  quaternion_type() );
        m_Bwcs = coordsys_type( vector3_type(-2,0,0), quaternion_type() );

      }
      break;
    case 'c':
      {
        OpenTissue::utility::Timer<real_type> watch;
        watch.start();
        types::result_type results;
        OpenTissue::collision::obb_tree_obb_tree<types>( m_Awcs, m_treeA, m_Bwcs, m_treeB, results);
        watch.stop();
        std::cout << "OBB Tree collision took " << watch() << " secs." << std::endl;
      }
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "build trees     [b]", 'b' );
    glutAddMenuEntry( "collision query [c]", 'c' );
    glutAddMenuEntry( "Decrease height [-]", '-' );
    glutAddMenuEntry( "Increase height [+]", '+' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "OBB Tree", controls );
  }

  void do_init(){}

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
