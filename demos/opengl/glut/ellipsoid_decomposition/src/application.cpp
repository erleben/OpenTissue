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


#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/geometry/geometry_ellipsoid.h>
#include <OpenTissue/core/geometry/geometry_ellipsoid_growing_fit.h>

#include <vector>


class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double,size_t>            math_types;
  typedef math_types::value_traits                                   value_traits;
  typedef math_types::vector3_type                                   vector3_type;

  typedef OpenTissue::polymesh::PolyMesh<math_types>                 mesh_type;
  typedef OpenTissue::mesh::CoordinateIterator<mesh_type>            coordinate_iterator;
  typedef OpenTissue::geometry::Ellipsoid<math_types>                ellipsoid_type;

  mesh_type                   m_mesh;
  ellipsoid_type              m_E;
  std::vector<ellipsoid_type> m_ellipsoids;

public:

  Application() { }

public:

  char const * do_get_title() const { return "Ellipsoid Decomposition Demo Application"; }

  void do_display()
  {
    OpenTissue::gl::ColorPicker(0.8,0.4,0.8);
    OpenTissue::gl::DrawMesh(m_mesh,GL_LINE_LOOP);
    OpenTissue::gl::ColorPicker(0.8,0.8,0.4);

    for(std::vector<ellipsoid_type>::iterator e=m_ellipsoids.begin();e!=m_ellipsoids.end();++e)
      OpenTissue::gl::DrawEllipsoid(*e, true);

    coordinate_iterator p ( m_mesh.vertex_begin() );
    coordinate_iterator p_end ( m_mesh.vertex_end() );
    OpenTissue::gl::ColorPicker(0.4,0.4,0.4);
    for(;p!=p_end;++p)
      OpenTissue::gl::DrawPoint(*p);
  }

  void do_action(unsigned char choice)
  {
    static mesh_type::vertex_iterator v = m_mesh.vertex_begin();

    switch(choice)
    {
    case '1':
      OpenTissue::mesh::make_box(1.0,1.0,1.0,m_mesh);
      v = m_mesh.vertex_begin();
      break;
    case '2':
      {
        std::vector<vector3_type> profile;
        profile.push_back(vector3_type(0.0,0.0,0.0));
        profile.push_back(vector3_type(5.0,0.0,5.0));
        profile.push_back(vector3_type(5.0,0.0,10.0));
        profile.push_back(vector3_type(0.0,0.0,15.0));
        OpenTissue::mesh::profile_sweep(
            profile.begin()
          , profile.end()
          , value_traits::two()*value_traits::pi()
          , 32
          , m_mesh
          );
        v = m_mesh.vertex_begin();
      }
      break;
    case '3':
      OpenTissue::mesh::make_disk(5.0,2.0,12,4,m_mesh);
      v = m_mesh.vertex_begin();
      break;
    case '4':
      OpenTissue::mesh::make_cylinder(5.0,2.0,12,m_mesh);
      v = m_mesh.vertex_begin();
      break;
    case '5':
      OpenTissue::mesh::make_sphere(5.0,12,5,m_mesh);
      v = m_mesh.vertex_begin();
      break;
    case '6':
      {
        std::vector<vector3_type> points(20);
        for(unsigned int i=0;i<20;++i)
          random(points[i]);
        OpenTissue::mesh::convex_hull(points.begin(),points.end(),m_mesh);
        v = m_mesh.vertex_begin();
      }
      break;
    case 't':
      {
        std::string data_path = opentissue_path;
        std::string filename = data_path + "/demos/data/obj/teapot.obj";
        OpenTissue::mesh::obj_read(filename,m_mesh);
        v = m_mesh.vertex_begin();
      }
      break;
    case 'e':
      {      
        mesh_type::vertex_iterator v_end = m_mesh.vertex_end();
        if(v==v_end)
          v = m_mesh.vertex_begin();

        m_ellipsoids.clear();
        coordinate_iterator begin ( m_mesh.vertex_begin() );
        coordinate_iterator end ( m_mesh.vertex_end() );
        OpenTissue::geometry::ellipsoid_growth_fit(
          v->m_coord
          , v->m_normal
          , begin
          , end
          , m_E
          );
        m_ellipsoids.push_back(m_E);
        ++v;
      }
      break;
    case 'E':
      {      
        m_ellipsoids.clear();

        mesh_type::vertex_iterator v_end = m_mesh.vertex_end();
        mesh_type::vertex_iterator v = m_mesh.vertex_begin();
        coordinate_iterator begin ( m_mesh.vertex_begin() );
        coordinate_iterator end ( m_mesh.vertex_end() );

        for(;v!=v_end;++v)
        {
          OpenTissue::geometry::ellipsoid_growth_fit(
            v->m_coord
            , v->m_normal
            , begin
            , end
            , m_E
            );
          m_ellipsoids.push_back(m_E);
        }
      }
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };// End Switch
    OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_mesh);
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu(menu);
    glutAddMenuEntry("box                               [1]", '1');
    glutAddMenuEntry("profile sweep                     [2]", '2');
    glutAddMenuEntry("disk                              [3]", '3');
    glutAddMenuEntry("cylinder                          [4]", '4');
    glutAddMenuEntry("sphere                            [5]", '5');
    glutAddMenuEntry("random convex hull                [6]", '6');
    glutAddMenuEntry("teapot                            [t]", 't');
    glutAddMenuEntry("Partial step-wise decomp          [e]", 'e');
    glutAddMenuEntry("Full Ellipsoid decomp             [E]", 'E');
    glutSetMenu(main_menu);
    glutAddSubMenu("ellipsoid decomposition", controls);
  }

  void do_init()
  {
    this->camera().move(80);
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
