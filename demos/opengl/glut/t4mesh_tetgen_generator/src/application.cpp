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
#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_tetgen_mesh_lofter.h>
#include <OpenTissue/core/containers/t4mesh/io/t4mesh_xml_read.h>
#include <OpenTissue/core/containers/t4mesh/io/t4mesh_xml_write.h>

class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;

  typedef math_types::vector3_type vector3_type;
  typedef math_types::real_type    real_type;

  OpenTissue::t4mesh::T4Mesh<> m_mesh;

  int m_max_objs;
  std::vector<std::string> m_objs;

  OpenTissue::t4mesh::mesh_lofter_settings m_config;

  bool m_quality;
  bool m_volume;
  bool m_quiet;
  GLenum m_draw_mode;
  double m_plane_height;
  bool m_draw_plane;

  bool polymesh2t4mesh(std::string const & in, std::string const & out)
  {
    OpenTissue::polymesh::PolyMesh<> manifold;
    // 2007-05-26 kenny please only use obj-format. Mesh-format is not going to be supported for long!
    if (!OpenTissue::mesh::default_read(in, manifold))
      if (!OpenTissue::mesh::obj_read(in, manifold)) 
      {
        std::cout << "ERROR: Could not import triangle mesh file \"" << in << "\"" << std::endl;
        return false;
      }

      vector3_type min_coord, max_coord, diff, center;

      OpenTissue::mesh::compute_mesh_center(manifold,center);
      center *= -1;
      OpenTissue::mesh::translate(manifold,center);

      OpenTissue::t4mesh::mesh_lofter_settings config = m_config;

      config.m_quality_ratio *= m_quality?1:0;
      config.m_maximum_volume *= m_volume?1:0;
      //  config.m_quality_ratio = sqrt(2.);
      //  config.m_quality_ratio = 0;
      //  config.m_maximum_volume = 0.0001;
      config.m_intermediate_file = "tmp";
      //  config.m_verify_input = true;
      config.m_quiet_output = m_quiet;
      if (!OpenTissue::t4mesh::mesh_lofter(m_mesh, manifold, config)) 
      {
        return false;
      }
      if (!OpenTissue::t4mesh::xml_write(out, m_mesh)) 
      {
        std::cout << "ERROR: Could not export tetrahedra mesh file \"" << out << "\"" << std::endl;
        return false;
      }
      return true;
  }


  bool get_mesh(std::string obj)
  {
    static std::string last_obj = "";

    if (obj.size() < 1 && last_obj.size() < 1) 
      return false;
    if (obj.size() > 0)
      last_obj = obj;
    else 
      obj = last_obj;

    std::string data_path = opentissue_path;

    size_t pos = obj.find_last_of(".");
    std::string ext = "obj";
    if (pos != std::string::npos) 
    {
      ext = obj.substr(pos+1);
      obj = obj.substr(0,pos);
    }
    if ("obj" == ext)
      data_path += "/demos/data/obj/";
    else if ("msh" == ext)
      data_path += "/mesh/";

    if (!polymesh2t4mesh(data_path+obj+"."+ext, obj+".xml"))
      return false;

    std::cout << "\n################################################################" << std::endl;
    std::cout << "Mesh: " << last_obj << std::endl;
    std::cout << "Quality ";
    if (m_quality)
      std::cout << "Ratio: " << m_config.m_quality_ratio;
    else
      std::cout << ": DISABLED!";
    std::cout << "\nVolumen (MAX) ";
    if (m_volume)
      std::cout << ": " << m_config.m_maximum_volume;
    else
      std::cout << ": DISABLED!";
    std::cout << "\n################################################################\n" << std::endl;

    return true;
  }

public:

  Application()  
  {
    m_max_objs = 8;
    m_objs.resize(m_max_objs);
    m_objs[0] = "torus.obj";
    m_objs[1] = "glass.obj";
    m_objs[2] = "knot.obj";
    m_objs[3] = "cow.obj";
    m_objs[4] = "jack.obj";
    m_objs[5] = "lamp.obj";
    m_objs[6] = "bunny_low_resolution.obj";
    m_objs[7] = "dragon_small_resolution.obj";
  }

public:

  char const * do_get_title() const { return "T4Mesh TetGen Generator Demo Application"; }

  void do_display()
  {
    OpenTissue::geometry::Plane<math_types> plane(vector3_type(0,-1,0), vector3_type(0,m_plane_height,0));
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glColor3d(1.0, 0.2, 0.4);
    OpenTissue::gl::DrawT4MeshCutThrough(m_mesh, plane, 1.0, false);
    glDisable(GL_LIGHTING);
    glColor3d(0.5, 0.1, 0.2);
    OpenTissue::gl::DrawT4MeshCutThrough(m_mesh, plane, 1.005, true);
    if(m_draw_plane) 
    {
      glColor3d(0.2, 0.2, 0.9);
      OpenTissue::gl::DrawPlane(plane, true);
    }
  }

  void do_action(unsigned char choice)
  {
    using std::sqrt;

    switch (choice)
    {
    case 'q':
      m_quality = !m_quality;
      get_mesh("");
      break;
    case 'Q':
      if (m_config.m_quality_ratio >= 2)
        m_config.m_quality_ratio = sqrt(2.);
      else
        m_config.m_quality_ratio = 2;
      if(m_quality)
        get_mesh("");
      break;
    case 'v':
      m_volume = !m_volume;
      if (m_config.m_maximum_volume <= 0)
        m_config.m_maximum_volume = 0.1;
      get_mesh("");
      break;
    case 'V':
      if (m_config.m_maximum_volume >= 0.1)
        m_config.m_maximum_volume = 0.01;
      else if (m_config.m_maximum_volume >= 0.01)
        m_config.m_maximum_volume = 0.001;
      else if (m_config.m_maximum_volume >= 0.001)
        m_config.m_maximum_volume = 0.0001;
      else if (m_config.m_maximum_volume >= 0.0001)
        m_config.m_maximum_volume = 0.00001;
      else
        m_config.m_maximum_volume = 0.1;
      if (m_volume)
        get_mesh("");
      break;
    case 'w':
      m_quiet = !m_quiet;
      break;
    case 'd':
      m_draw_mode = GL_POLYGON==m_draw_mode?GL_LINE_LOOP:GL_POLYGON;
      break;
    case 'p':
      m_draw_plane = !m_draw_plane;
      break;
    case '+':
      m_plane_height += 0.005;
      break;
    case '-':
      m_plane_height -= 0.005;
      break;
    default:
      int num = choice-'1';
      if (num >= 0 && num < m_max_objs)
        get_mesh(m_objs[num]);
      break;
    }
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int toggles = glutCreateMenu(menu);
    glutAddMenuEntry("Quality tetrahedral mesh [q]", 'q');
    glutAddMenuEntry("Change quality ratio [Q]", 'Q');
    glutAddMenuEntry("Volume constraint (Maximum) [v]", 'v');
    glutAddMenuEntry("Change maximum volume [V]", 'V');
    glutAddMenuEntry("Output from TetGen [w]", 'w');
    glutAddMenuEntry("Draw mode [d]", 'd');

    int mesh = glutCreateMenu(menu);
    for (int n = 0; n < m_max_objs; ++n) 
    {
      std::stringstream ss;
      ss << m_objs[n] << " [" << n+1 << "]";
      std::string s = ss.str();
      glutAddMenuEntry(s.c_str(), '1'+n);
    }

    glutSetMenu(main_menu);
    glutAddSubMenu("mesh", mesh);
    glutAddSubMenu("toggles", toggles);
  }

  void do_init()
  {
    this->camera().move(90);
    m_quality = m_config.m_quality_ratio > 0;
    m_volume  = m_config.m_maximum_volume > 0;
    m_quiet   = false;
    m_draw_mode = GL_POLYGON;
    m_plane_height = 0.0;
    m_draw_plane = true;
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
