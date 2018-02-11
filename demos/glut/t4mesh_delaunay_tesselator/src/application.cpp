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

#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_delaunay_tetrahedralization.h>

#include <iostream>

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
public:


  typedef OpenTissue::math::BasicMathTypes< double, size_t>   math_types;
  typedef math_types::vector3_type                            vector3_type;
  typedef std::vector< vector3_type >                         point_container;
  typedef OpenTissue::t4mesh::T4Mesh<math_types>              mesh_type;

protected:

  mesh_type         m_mesh;
  point_container   m_points;


public:


  Application() {  }

public:

  char const * do_get_title() const { return "Delaunay Tesselator Demo Application"; }

  void do_display()
  {
    OpenTissue::gl::ColorPicker( 0, 0, 1 );
    OpenTissue::gl::DrawPointsT4Mesh(m_points,m_mesh, 0.95, false);
  }

  void do_action(unsigned char choice)
  {
    switch ( choice )
    {
    case 't':
      {
        m_points.clear();
        m_points.resize( 20 );
        for ( point_container::iterator p = m_points.begin();p != m_points.end();++p )
          random(*p);
        OpenTissue::t4mesh::delaunay_tetrahedralization( m_points, m_mesh );
      }
      break;
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "Random point Triangulation [t]", 't' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "Tesselator", controls );
  }

  void do_init()
  {
    this->camera().move(90);
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
