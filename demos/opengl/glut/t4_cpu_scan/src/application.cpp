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

#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/graphics/core/gl/gl_cross_sections.h>
#include <OpenTissue/core/containers/mesh/mesh.h>

#include <OpenTissue/core/geometry/t4_cpu_scan/t4_cpu_scan.h>
#include <OpenTissue/utility/utility_timer.h>


class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;
  typedef OpenTissue::grid::Grid<float,math_types>               grid_type;
  typedef math_types::real_type                           real_type;
  typedef math_types::vector3_type                        vector3_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>      mesh_type;

  grid_type                           m_phi;
  mesh_type                          m_surface;
  mesh_type                          m_isosurface;

  OpenTissue::gl::CrossSections< grid_type > m_display_phi;

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.
  
  static unsigned char const C_KEY     = 'C';
  static unsigned char const S_KEY     = 'S';
  static unsigned char const G_KEY     = 'G';
  static unsigned char const NOT_KEY   = '!';
  static unsigned char const SHARP_KEY = '#';
  static unsigned char const QUOTE_KEY = '"';

protected:

  void doit(unsigned int const & choice)
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
    //--- read a mesh, center and scale it nicely
    OpenTissue::mesh::obj_read( meshfile, m_surface );
    OpenTissue::mesh::make_unit( m_surface );
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_surface);
    if(!is_manifold(m_surface))
      std::cout << "Yikes: Input mesh was degenerate" << std::endl;
    m_phi.create(vector3_type(-0.6,-0.6,-0.6),vector3_type(0.6,0.6,0.6), 256, 256, 256);
    //m_phi.create(vector3_type(-0.6,-0.6,-0.6),vector3_type(0.6,0.6,0.6), 128, 128, 128);
    m_display_phi.init(m_phi);
    real_type thickness = 0.1;

    OpenTissue::utility::Timer<double> watch;
    watch.start();
    OpenTissue::t4_cpu_scan(m_surface, thickness, m_phi, OpenTissue::t4_cpu_signed() );
    watch.stop();
    std::cout << "|V| = " 
      << m_surface.size_vertices() 
      << " |F| = " 
      << m_surface.size_faces() 
      << " done in " << watch() << std::endl;

    m_display_phi.invalidate();
    //std::cout << "--Done: phi min=" << min(m_phi) << ", max=" << max(m_phi) << std::endl;
  }



  class MyColorFunc
  {
  public:
    template <typename value_type>
    void operator()(
      value_type const & val
      , value_type const & /*min_val*/
      , value_type const & /*max_val*/
      , GLubyte & red
      , GLubyte & green
      , GLubyte & blue
      , GLubyte & alpha
      ) const
    {
      if(val<0)
      {
        red = 255;
        green = blue = 0;
        alpha = 100;
      }
      else if(val>0 && val<100)//--- just to catch unused!!!
      {
        green = red = 0;
        blue= 255;
        alpha = 100;
      }
      else
      {
        green = 255;
        red = blue= 0;
        alpha = 0;
      }
    }
  };


public:

  Application()  {  }

public:

  char const * do_get_title() const { return "T4 CPU Scan Demo Application"; }

  void do_display()
  {
    if(m_action[S_KEY])
    {
      OpenTissue::gl::ColorPicker(0.9, 0.3, 0.1, 1.0, GL_BACK);
      OpenTissue::gl::ColorPicker(0.1, 0.3, 0.9, 1.0, GL_FRONT);
      OpenTissue::gl::DrawMesh(m_surface,GL_LINE_LOOP);
      OpenTissue::gl::DrawMesh(m_isosurface);
    }
    if(m_action[G_KEY])
    {
      OpenTissue::gl::gl_check_errors("display Grey - start");
      m_display_phi.draw(OpenTissue::gl::GreyScaleColorFunctor());
    }
    if(m_action[C_KEY])
    {
      OpenTissue::gl::gl_check_errors("display Color - start");

      glEnable(GL_BLEND);
      glDisable(GL_LIGHTING);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      glDisable(GL_DEPTH_TEST);
      m_display_phi.draw(MyColorFunc());
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_LIGHTING);
      glDisable(GL_BLEND);
    }
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];

    switch ( choice )
    {
    case '0':   doit(0);    break;
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
    case '+':
      {
        if(m_action[NOT_KEY])
          m_display_phi.forward_i_plane();
        if(m_action[QUOTE_KEY])
          m_display_phi.forward_j_plane();
        if(m_action[SHARP_KEY])
          m_display_phi.forward_k_plane();
      }
      break;
    case '-':
      {
        if(m_action[NOT_KEY])
          m_display_phi.backward_i_plane();
        if(m_action[QUOTE_KEY])
          m_display_phi.backward_j_plane();
        if(m_action[SHARP_KEY])
          m_display_phi.backward_k_plane();
      }
      break;
    case 'G'://--- greyscale visualization
      {
        if(m_action[G_KEY])
          m_action[C_KEY] = false;
        m_display_phi.invalidate();
      }
      break;
    case 'C': //--- color visulization
      {
        if(m_action[C_KEY])
          m_action[G_KEY] = false;
        m_display_phi.invalidate();
      }
      break;
    case 'S':break; //--- Toggle Show Surface
    case 'E':
      {
        // extract isosurface of zero level set
        real_type lvl = 0.01;//--- KE 03-06-2003: Magic value, it works...
        m_isosurface.clear();
        OpenTissue::mesh::isosurface(m_phi,lvl,m_isosurface);
        OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_isosurface);
        OpenTissue::mesh::obj_write("isosurface.obj",m_isosurface);
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
    glutSetMenu( main_menu );
    glutAddSubMenu( "Test cases", controls );

    int toggles = glutCreateMenu( menu );
    glutAddMenuEntry( "Greyscale  [G]", 'G' );
    glutAddMenuEntry( "Color      [C]", 'C' );
    glutAddMenuEntry( "Surface    [S]", 'S' );
    glutAddMenuEntry( "Isosurface [E]", 'E' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "Drawing", toggles );
  }

  void do_init()
  {
    this->camera().move(95);
  }

  void do_run(){}

  void do_shutdown(){}

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
