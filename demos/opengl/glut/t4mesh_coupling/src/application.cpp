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
#include <OpenTissue/dynamics/fem/fem.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_mesh_coupling.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_block_generator.h>

#include <vector>

class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;
  typedef math_types::vector3_type                        vector3_type;
  typedef math_types::real_type                           real_type;
  typedef OpenTissue::fem::Mesh<math_types>               t4mesh_type;
  typedef OpenTissue::polymesh::PolyMesh<math_types>      polymesh_type;

  t4mesh_type                    m_volume_mesh;
  bool                           m_stiffness_warp_on;
  real_type                      m_gravity;

  polymesh_type                  m_surface_mesh;
  std::vector<vector3_type>      m_barycentric;
  std::vector<unsigned int >     m_bind_info;


protected:


  /**
  * This adapter is convient for accessing coordinate information in
  * a t4mesh as though it were stored in a stl vector or an array.
  */
  struct world_point_container
  {
    t4mesh_type * m_mesh;

    world_point_container(t4mesh_type * mesh) 
      : m_mesh(mesh) 
    {}

    vector3_type & operator[] (unsigned int const & idx)
    {
      return m_mesh->node(idx)->m_coord;
    }

    vector3_type const & operator[] (unsigned int const & idx)const
    {
      return m_mesh->node(idx)->m_coord;
    }

    void clear(){}
    unsigned int size()const{return m_mesh->size_nodes();}
    void resize(unsigned int){}

  };



public:

  Application(){}

public:

  char const * do_get_title() const { return "T4Mesh Coupling Demo Application"; }

  void do_display()
  {
    OpenTissue::gl::ColorPicker(0,0.5,0);
    OpenTissue::gl::DrawPointsT4Mesh(world_point_container(&m_volume_mesh),m_volume_mesh,0.95,true);

    OpenTissue::t4mesh::mesh_coupling::update_surface(m_surface_mesh,m_volume_mesh,m_barycentric,m_bind_info);

    OpenTissue::gl::ColorPicker(0,0,.5);
    OpenTissue::gl::DrawMesh(m_surface_mesh);
  }

  void do_action(unsigned char choice)
  {
    switch(choice)
    {
    case 's':
      m_stiffness_warp_on = !m_stiffness_warp_on;
      if(m_stiffness_warp_on)
        std::cout << "Stiffness warping on " << std::endl;
      else
        std::cout << "Stiffness warping off " << std::endl;
      break;
    case 'g':
      if(m_gravity>0)
      {
        m_gravity = 0;
        std::cout << "Gravity off" << std::endl;
      }
      else
      {
        m_gravity = 9.81;
        std::cout << "Gravity on" << std::endl;
      }
      break;
    case 'i':
      {
        using std::max;

        std::string data_path = opentissue_path;

        std::string meshfile = data_path + "/demos/data/obj/propella.obj";
        OpenTissue::mesh::obj_read( meshfile, m_surface_mesh );


        //--- Center mesh around orgi
        vector3_type center,min_coord,max_coord;
        OpenTissue::mesh::compute_mesh_center(m_surface_mesh,center);
        OpenTissue::mesh::translate(m_surface_mesh,-center);
        OpenTissue::mesh::compute_mesh_maximum_coord(m_surface_mesh,max_coord);
        OpenTissue::mesh::compute_mesh_minimum_coord(m_surface_mesh,min_coord);
        vector3_type diff = max_coord - min_coord;
        real_type extent = max(max(diff(0),diff(1)),diff(2));
        real_type s = 1./extent;
        OpenTissue::mesh::scale(m_surface_mesh,vector3_type(s,s,s));

        OpenTissue::mesh::compute_mesh_maximum_coord(m_surface_mesh,max_coord);
        OpenTissue::mesh::compute_mesh_minimum_coord(m_surface_mesh,min_coord);

        diff = max_coord - min_coord;
        center = diff * .5 + vector3_type(0.01);
        OpenTissue::mesh::translate(m_surface_mesh,center);

        real_type young = 500000;
        real_type poisson = 0.33;
        real_type density = 1000;
        {
          world_point_container point_wrapper(&m_volume_mesh);

          unsigned int I = 8;
          unsigned int J = 8;
          unsigned int K = 8;

          real_type slack = 0.02;
          real_type width = (diff(0)+slack)/I;
          real_type height = (diff(1)+slack)/J;
          real_type depth = (diff(2)+slack)/K;

          OpenTissue::t4mesh::generate_blocks(I,J,K,width,height,depth,m_volume_mesh);
          OpenTissue::fem::update_original_coord(m_volume_mesh.node_begin(),m_volume_mesh.node_end());
          t4mesh_type::node_iterator node = m_volume_mesh.node_begin();
          for ( ; node != m_volume_mesh.node_end(); ++node)
          {
            if(node->m_model_coord(0) < 0.01)
              node->m_fixed = true;
          }
          real_type c_yield = 10e30;     //--- These plasticity settings means that plasticity is turned off
          real_type c_creep = 0;
          real_type c_max = 0;
          OpenTissue::fem::init(m_volume_mesh,young,poisson,density,c_yield,c_creep,c_max);
        }

        OpenTissue::t4mesh::mesh_coupling::bind_surface(m_surface_mesh,m_volume_mesh,m_barycentric,m_bind_info);
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
    glutAddMenuEntry("Toggle Stiffness warping [s]", 's');
    glutAddMenuEntry("Toggle gravity [g]", 'g');
    glutSetMenu(main_menu);
    glutAddSubMenu("t4mesh coupling", controls);
  }

  void do_init()
  {
    this->camera().move(90);
  }

  void do_run()
  {
    // Calculate external forces acting on the model.
    // The external forces are stored in each vertex and consists of a downward gravity.
    // If a vertex is being dragged by the user, its velocity vector is added as an external force.
    t4mesh_type::node_iterator node = m_volume_mesh.node_begin();
    for ( ; node != m_volume_mesh.node_end(); ++node)
      node->m_f_external = vector3_type(0.0, -(node->m_mass * m_gravity), 0.0);

    OpenTissue::fem::simulate(m_volume_mesh,0.01,m_stiffness_warp_on);
  }

  void do_shutdown(){}

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
