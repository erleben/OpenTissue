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

#include <OpenTissue/core/containers/mesh/mesh.h>

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
protected:

  bool m_action[ 256 ];          ///< Boolean array used to keep track of the user actions/selections.
  
  static unsigned char const H_KEY = 'H';
  static unsigned char const G_KEY = 'G';
  

  typedef OpenTissue::polymesh::PolyMesh<> mesh_type;
  typedef mesh_type::math_types            math_types;
  typedef math_types::vector3_type         vector3_type;

  mesh_type   m_mesh;

protected:

  void test_case( unsigned int const & choice )
  {
    std::string data_path = opentissue_path;
    std::string meshfile;
    switch ( choice )
    {
    case 0:  meshfile = data_path + "/demos/data/obj/box.obj";              break;
    case 1:  meshfile = data_path + "/demos/data/obj/cylinder.obj";         break;
    case 2:  meshfile = data_path + "/demos/data/obj/pointy.obj";           break;
    case 3:  meshfile = data_path + "/demos/data/obj/diku.obj";         break;
    case 4:  meshfile = data_path + "/demos/data/obj/tube.obj";             break;
    case 5:  meshfile = data_path + "/demos/data/obj/sphere.obj";      break;
    case 6:  meshfile = data_path + "/demos/data/obj/teapot.obj";           break;
    case 7:  meshfile = data_path + "/demos/data/obj/propella.obj";    break;
    case 8:  meshfile = data_path + "/demos/data/obj/funnel.obj";           break;
    case 9:  meshfile = data_path + "/demos/data/obj/dragon_small_resolution.obj";      break;
    case 10: meshfile = data_path + "/demos/data/obj/cow.obj";         break;
    case 11: meshfile = data_path + "/demos/data/obj/bend.obj";             break;
    case 12: meshfile = data_path + "/demos/data/obj/bowl.obj";             break;
    case 13: meshfile = data_path + "/demos/data/obj/torus.obj";            break;
    case 14: meshfile = data_path + "/demos/data/obj/bunny_medium_resolution.obj";       break;
    case 15: meshfile = data_path + "/demos/data/obj/knot.obj";             break;
    case 16: meshfile = data_path + "/demos/data/obj/bunny_high_resolution.obj";      break;
    case 17: meshfile = data_path + "/demos/data/obj/dragon_high_resolution.obj";     break;
    case 18: meshfile = data_path + "/demos/data/obj/buddha.obj";     break;
    };
    OpenTissue::mesh::obj_read( meshfile, m_mesh );
    OpenTissue::mesh::make_unit(m_mesh);
  }


public:

  Application()  {  }

public:

  char const * do_get_title() const { return "Vertex Mean Curvature Demo Application"; }

  void do_display()
  {
    if(m_action[H_KEY])
    {
      OpenTissue::gl::ColorPicker(1.0,0.0,0.0);
      mesh_type::vertex_iterator vend    = m_mesh.vertex_end();
      mesh_type::vertex_iterator vertex = m_mesh.vertex_begin();
      for(;vertex!=vend;++vertex)
        OpenTissue::gl::DrawVector( vertex->m_coord, vertex->m_normal, 0.5);
      OpenTissue::gl::ColorPicker(0.0,0.0,1.0);
    }
    glDisable( GL_LIGHTING ) ;
    OpenTissue::gl::DrawMesh(m_mesh,GL_LINE_LOOP);
    glEnable( GL_LIGHTING ) ;
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];

    static unsigned int last_choice = 1;

    switch ( choice )
    {
    case '0':   test_case(0);   last_choice = 0; break;
    case '1':   test_case(1);   last_choice = 1; break;
    case '2':   test_case(2);   last_choice = 2; break;
    case '3':   test_case(3);   last_choice = 3; break;
    case '4':   test_case(4);   last_choice = 4; break;
    case '5':   test_case(5);   last_choice = 5; break;
    case '6':   test_case(6);   last_choice = 6; break;
    case '7':   test_case(7);   last_choice = 7; break;
    case '8':   test_case(8);   last_choice = 8; break;
    case '9':   test_case(9);   last_choice = 9; break;
    case 'a':   test_case(10);  last_choice = 10;  break;
    case 'b':   test_case(11);  last_choice = 11;  break;
    case 'c':   test_case(12);  last_choice = 12;  break;
    case 'd':   test_case(13);  last_choice = 13;  break;
    case 'e':   test_case(14);  last_choice = 14;  break;
    case 'f':   test_case(15);  last_choice = 15;  break;
    case 'g':   test_case(16);  last_choice = 16;  break;
    case 'h':   test_case(17);  last_choice = 17;  break;
    case 'i':   test_case(18);  last_choice = 18;  break;
    case 'H':
      {
        mesh_type::vertex_iterator end    = m_mesh.vertex_end();
        mesh_type::vertex_iterator vertex = m_mesh.vertex_begin();
        for(;vertex!=end;++vertex)
        {
          OpenTissue::polymesh::compute_vertex_mean_curvature_normal( *vertex, vertex->m_color);
          vertex->m_normal = unit(vertex->m_color)*0.1;
        }
        m_action[H_KEY] = true;
      }
      break;
    case 'G':
      {
        mesh_type::vertex_iterator end    = m_mesh.vertex_end();
        mesh_type::vertex_iterator vertex = m_mesh.vertex_begin();
        for(;vertex!=end;++vertex)
        {
          float tst = 0.0;
          OpenTissue::polymesh::compute_vertex_gaussian_curvature( *vertex, tst );
          vertex->m_normal *= tst;
        }
        m_action[G_KEY] = true;
      }
      break;
    case 'S':
      {
        OpenTissue::polymesh::subdivide(m_mesh, 0.0);
        OpenTissue::polymesh::triangulate(m_mesh);
        m_action[H_KEY] = false;
      }
      break;
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };// End Switch
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu(menu);
    glutAddMenuEntry( "Test case  0 (box)         [0]", '0' );
    glutAddMenuEntry( "Test case  1 (cylinder)    [1]", '1' );
    glutAddMenuEntry( "Test case  2 (pointy)      [2]", '2' );
    glutAddMenuEntry( "Test case  3 (diku)        [3]", '3' );
    glutAddMenuEntry( "Test case  4 (tube)        [4]", '4' );
    glutAddMenuEntry( "Test case  5 (sphere)      [5]", '5' );
    glutAddMenuEntry( "Test case  6 (teapot)      [6]", '6' );
    glutAddMenuEntry( "Test case  7 (propella)    [7]", '7' );
    glutAddMenuEntry( "Test case  8 (funnel)      [8]", '8' );
    glutAddMenuEntry( "Test case  9 (dragon)      [9]", '9' );
    glutAddMenuEntry( "Test case 10 (cow)         [a]", 'a' );
    glutAddMenuEntry( "Test case 11 (bend)        [b]", 'b' );
    glutAddMenuEntry( "Test case 12 (bowl)        [c]", 'c' );
    glutAddMenuEntry( "Test case 13 (torus)       [d]", 'd' );
    glutAddMenuEntry( "Test case 14 (bunny_3851)  [e]", 'e' );
    glutAddMenuEntry( "Test case 15 (knot)        [f]", 'f' );
    glutAddMenuEntry( "Test case 16 (bunny_16k)   [g]", 'g' );
    glutAddMenuEntry( "Test case 17 (dragon_47k)  [h]", 'h' );
    glutAddMenuEntry( "Test case 18 (buddha_67k)  [i]", 'i' );
    glutAddMenuEntry("compute vertex mean curvature normals [H]", 'H');
    glutAddMenuEntry("compute vertex gaussian curvature     [G]", 'G');
    glutAddMenuEntry("subdivide mesh                        [S]", 'S');
    glutSetMenu(main_menu);
    glutAddSubMenu("curvature", controls);
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
