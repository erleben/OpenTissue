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

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/collision/sdf/sdf.h>
#include <OpenTissue/collision/sdf/sdf_debug_draw_sampling.h>
#include <OpenTissue/collision/sdf/sdf_debug_draw_bvh.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/utility/utility_timer.h>

class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double, size_t>  math_types;
  typedef math_types::real_type                             real_type;
  typedef math_types::vector3_type                          vector3_type;
  typedef math_types::coordsys_type                         coordsys_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>        mesh_type;
  typedef OpenTissue::grid::Grid<float,math_types>                 grid_type;
  typedef OpenTissue::sdf::Geometry<mesh_type,grid_type>     sdf_geometry_type;

  class contact_point_type
  {
  public:
    vector3_type  m_n;
    vector3_type  m_p;
    real_type     m_distance;
  };

  typedef std::vector< contact_point_type > contact_point_container;

  contact_point_container m_contacts;

  coordsys_type m_wcsA;
  coordsys_type m_wcsB;

  sdf_geometry_type  m_A;
  sdf_geometry_type  m_B;

  typedef OpenTissue::geometry::Sphere<math_types> sphere_type;
  typedef OpenTissue::geometry::Plane<math_types>  plane_type;

  sphere_type m_sphere;
  plane_type  m_plane;

  unsigned int m_depth;      ///< Used for debug drawing, indicates the depth of spheres in bvh of sdf_geometry type
  bool m_action[256];               ///< Boolean array used to keep track of the user actions/selections.

  static unsigned char const ONE_KEY   = '1';
  static unsigned char const TWO_KEY   = '2';
  static unsigned char const THREE_KEY = '3';
  static unsigned char const FOUR_KEY  = '4';
  static unsigned char const FIVE_KEY  = '5';
  static unsigned char const S_KEY     = 'S';
  static unsigned char const B_KEY     = 'B';

public:

  Application() {  }

public:

  char const * do_get_title() const { return "Signed Distance Field Collision Detection Demo Application"; }

  void do_display()
  {
    OpenTissue::gl::ColorPicker(0.8,0.1,0.1);
    for(contact_point_container::iterator cp = m_contacts.begin(); cp != m_contacts.end();++cp)
    {
      vector3_type n = cp->m_n*0.25;
      OpenTissue::gl::DrawVector( cp->m_p, n, 0.1 );
    }
    //--- Draw object A
    if(m_action[ONE_KEY])
    {
      glPushMatrix();
      OpenTissue::gl::ColorPicker(0.8,0.4,0.1);
      OpenTissue::gl::Transform(m_wcsA);
      OpenTissue::gl::DrawMesh(m_A.m_mesh,GL_LINE_LOOP);
      if(m_action[S_KEY])
      {
        OpenTissue::gl::ColorPicker(0.8, 0.1, 0.8);
        OpenTissue::sdf::debug_draw_sampling(m_A);
      }
      if(m_action[B_KEY])
      {
        OpenTissue::gl::ColorPicker(0.8,0.4,0.1);
        OpenTissue::sdf::debug_draw_bvh(m_A, m_depth);
      }
      glPopMatrix();
    }
    if(m_action[TWO_KEY])
    {
      glPushMatrix();
      OpenTissue::gl::ColorPicker(0.8,0.4,0.1);
      OpenTissue::gl::Transform(m_wcsA);
      OpenTissue::gl::DrawSphere(m_sphere, true);
      glPopMatrix();
    }
    if(m_action[THREE_KEY])
    {
      glPushMatrix();
      OpenTissue::gl::ColorPicker(0.8,0.4,0.1);
      OpenTissue::gl::Transform(m_wcsA);
      OpenTissue::gl::DrawPlane(m_plane, true);
      glPopMatrix();
    }
    //--- Draw object B
    if(m_action[ONE_KEY] || m_action[TWO_KEY] || m_action[THREE_KEY])
    {
      glPushMatrix();
      OpenTissue::gl::ColorPicker(0.1,0.4,0.8);
      OpenTissue::gl::Transform(m_wcsB);
      OpenTissue::gl::DrawMesh(m_B.m_mesh,GL_LINE_LOOP);
      if(m_action[S_KEY])
      {
        OpenTissue::gl::ColorPicker(0.8, 0.1, 0.8);
        OpenTissue::sdf::debug_draw_sampling(m_B);
      }
      if(m_action[B_KEY])
      {
        OpenTissue::gl::ColorPicker(0.8,0.4,0.1);
        OpenTissue::sdf::debug_draw_bvh(m_B, m_depth);
      }
      glPopMatrix();
    }
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];
    switch ( choice )
    {
    case 'i':
      {
        std::string datapath = opentissue_path;
        std::string meshfile;

        meshfile = datapath + "/demos/data/obj/pointy.obj";
        mesh_type mesh;
        OpenTissue::mesh::obj_read( meshfile, mesh );
        OpenTissue::polymesh::triangulate(mesh);

        double edge_resolution = 0.01;
        bool face_sampling = true;
        OpenTissue::sdf::semiauto_init_geometry(mesh,edge_resolution,face_sampling,m_A);
        OpenTissue::sdf::semiauto_init_geometry(mesh,edge_resolution,face_sampling,m_B);

        m_wcsA.identity();
        m_wcsA.T() += vector3_type(1.0,0.0,0.0);

        m_wcsB.identity();
        m_wcsB.T() += vector3_type(-1.0,0.0,0.0);
      }
      break;
    case '1':
      {

        coordsys_type::vector3_type T;
        coordsys_type::quaternion_type Q;

        OpenTissue::math::random(T,-0.5,0.5);
        Q.random();
        m_wcsA = coordsys_type(T,Q);

        OpenTissue::math::random(T,-0.5,0.5);
        Q.random();
        m_wcsB = coordsys_type(T,Q);

        OpenTissue::utility::Timer<real_type> watch;
        watch.start();
        OpenTissue::collision::sdf_sdf(m_wcsA,m_A,m_wcsB,m_B,m_contacts,0.01);
        watch.stop();

        std::cout << "|C| = " << m_contacts.size() << std::endl;
        std::cout << "collision query took " << watch() << " seconds" << std::endl;

        m_action[ONE_KEY] = true;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
      }
      break;
    case '2':
      {
        m_sphere.center(vector3_type(0,0,0));
        m_sphere.radius(0.1);

        coordsys_type::vector3_type T;
        coordsys_type::quaternion_type Q;

        OpenTissue::math::random(T,-0.5,0.5);
        Q.random();
        m_wcsA = coordsys_type(T,Q);

        OpenTissue::math::random(T,-0.5,0.5);
        Q.random();
        m_wcsB = coordsys_type(T,Q);

        OpenTissue::utility::Timer<real_type> watch;
        watch.start();
        OpenTissue::collision::sphere_sdf(m_wcsA, m_sphere, m_wcsB, m_B, m_contacts,0.01);
        watch.stop();

        std::cout << "|C| = " << m_contacts.size() << std::endl;
        std::cout << "collision query took " << watch() << " seconds" << std::endl;

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = true;
        m_action[THREE_KEY] = false;
      }
      break;
    case '3':
      {
        m_wcsA.T().clear();
        m_wcsA.Q().identity();

        m_plane.w() = -0.4;
        m_plane.n() = vector3_type(0,1,0);

        coordsys_type::vector3_type T;
        coordsys_type::quaternion_type Q;
        OpenTissue::math::random(T,-0.5,0.5);
        Q.random();
        m_wcsB = coordsys_type(T,Q);

        OpenTissue::utility::Timer<real_type> watch;
        watch.start();
        OpenTissue::collision::plane_sdf( m_wcsA, m_plane, m_wcsB, m_B, m_contacts, 0.01);
        watch.stop();

        std::cout << "|C| = " << m_contacts.size() << std::endl;
        std::cout << "collision query took " << watch() << " seconds" << std::endl;

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = true;
      }
      break;
    case 'B':break; //--- Toggle show BVH debug drawing
    case 'S':break; //--- Toggle show sampling debug drawing
    case '+':  //--- Increase BVH depth in debug drawing
      ++m_depth;
      std::cout << "depth = " << m_depth << std::endl;
      break;
    case '-': //--- Decrease BVH depth in debug drawing
      if(m_depth>0)
        --m_depth;
      std::cout << "depth = " << m_depth << std::endl;
      break;
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu( menu );
    glutAddMenuEntry( "Initialize SDF geometries           [i]", 'i' );

    glutAddMenuEntry( "sdf sdf collision query             [1]", '1' );
    glutAddMenuEntry( "sphere sdf collision query          [2]", '2' );
    glutAddMenuEntry( "plane sdf collision query           [3]", '3' );

    glutAddMenuEntry( "Toggle show BVH debug drawing       [B]", 'B' );
    glutAddMenuEntry( "Toggle show sampling debug drawing  [S]", 'S' );
    glutAddMenuEntry( "Increase BVH depth in debug drawing [+]", '+' );
    glutAddMenuEntry( "Decrease BVH depth in debug drawing [-]", '-' );
    glutSetMenu( main_menu );
    glutAddSubMenu( "signed distance field", controls );
  }

  void do_init()
  {
    m_depth = 0;
    this->camera().move(90);
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
