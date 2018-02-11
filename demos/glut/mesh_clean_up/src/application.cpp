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

  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;
  typedef math_types::real_type                           real_type;
  typedef math_types::vector3_type                        vector3_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>      mesh_type;

  mesh_type                          m_input;
  mesh_type                          m_output;

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.
  
  static unsigned char const I_KEY = 'I';

protected:

  void doit(unsigned int const & choice)
  {
    std::string data_path = opentissue_path;
    std::string meshfile;
    switch ( choice )
    {

    case  1: meshfile = data_path + "/demos/data/obj/bend.obj";                       break;
    case  2: meshfile = data_path + "/demos/data/obj/bowl.obj";                       break;
    case  3: meshfile = data_path + "/demos/data/obj/box.obj";                        break;
    case  4: meshfile = data_path + "/demos/data/obj/buddha.obj";                     break;
    case  5: meshfile = data_path + "/demos/data/obj/bunny_high_resolution.obj";      break;
    case  6: meshfile = data_path + "/demos/data/obj/bunny_low_resolution.obj";       break;
    case  7: meshfile = data_path + "/demos/data/obj/bunny_medium_resolution.obj";    break;
    case  8: meshfile = data_path + "/demos/data/obj/cow.obj";                        break;
    case  9: meshfile = data_path + "/demos/data/obj/cylinder.obj";                   break;
    case 10: meshfile = data_path + "/demos/data/obj/diku.obj";                       break;
    case 11: meshfile = data_path + "/demos/data/obj/dragon_high_resolution.obj";     break;
    case 12: meshfile = data_path + "/demos/data/obj/dragon_small_resolution.obj";    break;
    case 13: meshfile = data_path + "/demos/data/obj/funnel.obj";                     break;
    case 14: meshfile = data_path + "/demos/data/obj/glass.obj";                      break;
    case 15: meshfile = data_path + "/demos/data/obj/jack.obj";                       break;
    case 16: meshfile = data_path + "/demos/data/obj/knot.obj";                       break;
    case 17: meshfile = data_path + "/demos/data/obj/lamp.obj";                       break;
    case 18: meshfile = data_path + "/demos/data/obj/pointy.obj";                     break;
    case 19: meshfile = data_path + "/demos/data/obj/propella.obj";                   break;
    case 20: meshfile = data_path + "/demos/data/obj/sphere.obj";                     break;
    case 21: meshfile = data_path + "/demos/data/obj/support_box.obj";                break;
    case 22: meshfile = data_path + "/demos/data/obj/teapot.obj";                     break;
    case 23: meshfile = data_path + "/demos/data/obj/torus.obj";                      break;
    case 24: meshfile = data_path + "/demos/data/obj/tube.obj";                       break;
    };

    OpenTissue::mesh::obj_read( meshfile, m_input );
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_input);

    m_output = m_input;

    OpenTissue::mesh::make_unit( m_output );
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_output);

    if(!is_manifold(m_output))
    {
      std::cout << "Degenerate mesh: trying to nicefy it" << std::endl;
      mesh_type tmp(m_output);
      OpenTissue::mesh::remove_redundant_vertices(tmp,m_output,10e-10);
    }
    if(!is_manifold(m_output))
    {
      std::cout << "Degenerate mesh: trying to patch it" << std::endl;
      OpenTissue::polymesh::naive_patcher(m_output);
    }
    if(!is_manifold(m_output))
      std::cout << "Degenerate mesh: giving up" << std::endl;

    OpenTissue::polymesh::triangulate( m_output );

    OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_input);
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_output);

    //if(m_action['W'])
    //  OpenTissue::mesh::obj_write( meshfile, m_output );
  }

public:

  Application(){}

public:

  char const * do_get_title() const { return "Mesh Clean Up Demo Application"; }

  void do_display()
  {
    OpenTissue::geometry::AABB<math_types> aabb(-0.5,-0.5,-0.5,0.5,0.5,0.5);
    OpenTissue::gl::ColorPicker(1.0, 0.0, 0.0);
    OpenTissue::gl::DrawAABB( aabb, true);
    if(m_action[I_KEY])
    {
      OpenTissue::gl::ColorPicker(0.3, 0.9, 0.3);
      OpenTissue::gl::DrawMesh(m_input);
    }
    else
    {
      OpenTissue::gl::ColorPicker(0.3, 0.3, 0.9);
      OpenTissue::gl::DrawMesh(m_output);
    }
  }

  void do_action(unsigned char choice)
  {
    m_action[choice] = ! m_action[choice];

    switch ( choice )
    {
    case '1':   doit(1);    break;
    case '2':   doit(2);    break;
    case '3':   doit(3);    break;
    case '4':   doit(4);    break;
    case '5':   doit(5);    break;
    case '6':   doit(6);    break;
    case '7':   doit(7);    break;
    case '8':   doit(8);    break;
    case '9':   doit(9);    break;
    case 'a':   doit(10);    break;
    case 'b':   doit(11);    break;
    case 'c':   doit(12);    break;
    case 'd':   doit(13);    break;
    case 'e':   doit(14);    break;
    case 'f':   doit(15);    break;
    case 'g':   doit(16);    break;
    case 'h':   doit(17);    break;
    case 'i':   doit(18);    break;
    case 'j':   doit(19);    break;
    case 'k':   doit(20);    break;
    case 'n':   doit(21);    break;
    case 'p':   doit(22);    break;
    case 'r':   doit(23);    break;
    case 's':   doit(24);    break;
    case 'I':
      if(m_action[I_KEY])
        std::cout<< "showing input mesh" << std::endl;
      else
        std::cout<< "showing output mesh" << std::endl;
      break;
    //case 'W':
    //  if(m_action['W'])
    //    std::cout<< "Overwrite ON" << std::endl;
    //  else
    //    std::cout<< "Overwrite OFF" << std::endl;
    //  break;
    default:    
      std::cout << "You pressed " << choice << std::endl;
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "bend.obj                    [1]", '1' );
    glutAddMenuEntry( "bowl.obj                    [2]", '2' );
    glutAddMenuEntry( "box.obj                     [3]", '3' );
    glutAddMenuEntry( "buddha.obj                  [4]", '4' );
    glutAddMenuEntry( "bunny_high_resolution.obj   [5]", '5' );
    glutAddMenuEntry( "bunny_low_resolution.obj    [6]", '6' );
    glutAddMenuEntry( "bunny_medium_resolution.obj [7]", '7' );
    glutAddMenuEntry( "cow.obj                     [8]", '8' );
    glutAddMenuEntry( "cylinder.obj                [9]", '9' );
    glutAddMenuEntry( "diku.obj                    [a]", 'a' );
    glutAddMenuEntry( "dragon_high_resolution.obj  [b]", 'b' );
    glutAddMenuEntry( "dragon_small_resolution.obj [c]", 'c' );
    glutAddMenuEntry( "funnel.obj                  [d]", 'd' );
    glutAddMenuEntry( "glass.obj                   [e]", 'e' );
    glutAddMenuEntry( "jack.obj                    [f]", 'f' );
    glutAddMenuEntry( "knot.obj                    [g]", 'g' );
    glutAddMenuEntry( "lamp.obj                    [h]", 'h' );
    glutAddMenuEntry( "pointy.obj                  [i]", 'i' );
    glutAddMenuEntry( "propella.obj                [j]", 'j' );
    glutAddMenuEntry( "sphere.obj                  [k]", 'k' );
    glutAddMenuEntry( "support_box.obj             [n]", 'n' );
    glutAddMenuEntry( "teapot.obj                  [p]", 'p' );
    glutAddMenuEntry( "torus.obj                   [r]", 'r' );
    glutAddMenuEntry( "tube.obj                    [s]", 's' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "Test cases", controls );

    int toggles = glutCreateMenu( menu );
    glutAddMenuEntry( "Toggle input/output mesh    [I]", 'I' );
    //glutAddMenuEntry( "Toggle overwrite input mesh [W]", 'W' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "Toggles", toggles );
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
