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
#include <OpenTissue/core/containers/t4mesh/t4mesh.h>
#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/core/containers/grid/util/grid_voxelizer.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_block_generator.h>

class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double,size_t>  math_types;
  typedef math_types::vector3_type                         vector3_type;
  typedef OpenTissue::t4mesh::T4Mesh<>                     t4mesh_type;
  typedef OpenTissue::polymesh::PolyMesh<>                 mesh_type;

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.
  
  static unsigned char const S_KEY = 'S';
  static unsigned char const T_KEY = 'T';
  

  mesh_type         m_mesh;
  t4mesh_type       m_t4mesh;

public:

  Application()  {  }

public:

  char const * do_get_title() const { return "T4Mesh Block Generator Demo Application"; }

  void do_display()
  {
    if(m_action[T_KEY])
    {
      OpenTissue::gl::ColorPicker(0.1,0.8,0.4);
      OpenTissue::gl::DrawT4Mesh(m_t4mesh,0.95,true);
      for(t4mesh_type::node_iterator n = m_t4mesh.node_begin(); n != m_t4mesh.node_end(); ++n)
      {
        OpenTissue::gl::DrawPoint(n->m_coord, 0.01);
      }
    }
    if(m_action[S_KEY])
    {
      OpenTissue::gl::ColorPicker(0.1,0.4,.8);
      OpenTissue::gl::DrawMesh(m_mesh);
    }
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];
    switch(choice)
    {
    case 'T':break; //--- Toggle show tetrahedra
    case 'S':break; //--- Toggle show surface mesh
    case 'i':
      {
        std::string datapath = opentissue_path;

        std::string meshfile = datapath + "/demos/data/obj/torus.obj";
        OpenTissue::mesh::obj_read( meshfile, m_mesh );

        OpenTissue::mesh::make_unit(m_mesh);
        OpenTissue::mesh::compute_angle_weighted_vertex_normals(m_mesh);

        vector3_type min_coord, max_coord;
        OpenTissue::mesh::compute_mesh_maximum_coord(m_mesh,max_coord);
        OpenTissue::mesh::compute_mesh_minimum_coord(m_mesh,min_coord);
        vector3_type diff = max_coord - min_coord;
        min_coord -= diff*0.1;
        max_coord += diff*0.1;

        unsigned int I = 32;
        unsigned int J = 32;
        unsigned int K = 32;

        OpenTissue::grid::Grid<float,math_types> voxels;
        voxels.create( min_coord,max_coord, I, J, K );
        OpenTissue::grid::voxelizer( m_mesh, voxels );
        OpenTissue::t4mesh::generate_blocks(voxels,m_t4mesh);
        m_action[S_KEY] = true;
        m_action[T_KEY] = true;
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
    glutAddMenuEntry("Inititialize [i]", 'i');
    glutAddMenuEntry("Toggle show surface mesh [S]", 'S');
    glutAddMenuEntry("Toggle show tetrahedra [T]", 'T');
    glutSetMenu(main_menu);
    glutAddSubMenu("block generator", controls);
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
