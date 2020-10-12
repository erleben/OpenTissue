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
#include <OpenTissue/dynamics/meshless_deformation/meshless_deformation.h>
#include <OpenTissue/core/containers/mesh/mesh.h>

class Application 
  : public OpenTissue::graphics::PerspectiveViewApplication
{
public:

  typedef OpenTissue::math::BasicMathTypes<double, size_t> math_types;
  typedef OpenTissue::polymesh::PolyMesh<math_types>       mesh_type;
  typedef math_types::real_type                            real_type;
  typedef math_types::vector3_type                         vector3_type;

  typedef OpenTissue::meshless_deformation::ShapeMatchingSimulator<math_types>   simulation_method;

protected:

  real_type           m_gravity;        ///< Acceleration of gravity.
  simulation_method   m_simulator;         ///< The simulation method...
  mesh_type           m_mesh;           ///< A mesh to deform!!!
  simulation_method::particle_type * m_pull_me;   ///< Particle being pulled by external force.
  vector3_type     m_pull_force;   ///< Force to pull with.

public:

  Application() 
    : m_gravity( 0.0 )
    , m_pull_me(0) 
    , m_pull_force( 0.0, 0.0, 0.0 )
  {  }

public:

  char const * do_get_title() const { return "Meshless Shape Matching Demo Application"; }

  void do_display()
  {
    simulation_method::particle_iterator begin = m_simulator.particle_begin();
    simulation_method::particle_iterator end   = m_simulator.particle_end();
    simulation_method::particle_iterator particle;
    for (particle=begin;particle!=end; ++particle)
    {
      if(particle->fixed())
        OpenTissue::gl::ColorPicker(0.0,0.0,1.0);
      else
        OpenTissue::gl::ColorPicker(1.0,0.0,0.0);
      OpenTissue::gl::DrawPoint(particle->x(),0.05);
    }
    if(m_pull_me)
    {
      OpenTissue::gl::ColorPicker(0.0,1.0,0.0);
      OpenTissue::gl::DrawVector(m_pull_me->x(),m_pull_force,0.05);
    }
  }

  void do_action(unsigned char choice)
  {
    switch(choice)
    {
    case 's':      this->run();    break;
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
        m_simulator.clear();
        m_mesh.clear();

        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/propella.obj";
        OpenTissue::mesh::obj_read( meshfile, m_mesh );

        vector3_type m,M;

        OpenTissue::mesh::compute_mesh_minimum_coord(m_mesh,m);
        OpenTissue::mesh::compute_mesh_maximum_coord(m_mesh,M);

        std::cout << " min coord = " << m << std::endl;
        std::cout << " max coord = " << M << std::endl;

        simulation_method::cluster_type * cluster = m_simulator.create_cluster();
        mesh_type::vertex_iterator vertex = m_mesh.vertex_begin();
        mesh_type::vertex_iterator end    = m_mesh.vertex_end();
        m_pull_me = 0;
        for(;vertex!=end;++vertex)
        {
          simulation_method::particle_type * particle = m_simulator.create_particle();
          particle->bind(vertex->m_coord);
          cluster->bind_particle(*particle);
          if(vertex->m_coord(0) < -0.499)
            particle->fixed() = true;
          if(vertex->m_coord(0) > 0.499)
            m_pull_me = &(*particle);
        }

        cluster->set_beta(0.25);
        cluster->set_tau( 0.01 /8.0 );

        cluster->set_yield(0.09); // to turn of set value to value_traits::infinity()
        cluster->set_creep(0.01);
        cluster->set_max(0.10);

        m_pull_force = vector3_type(0.0, 50.0, 0.0);
        m_simulator.init();
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
    glutAddMenuEntry("Toggle gravity [g]", 'g');
    glutSetMenu(main_menu);
    glutAddSubMenu("shape matching", controls);
  }

  void do_init()
  {
    this->camera().move(95);
  }

  void do_run()
  {
    real_type dt = 0.01;
    {
      // Calculate external forces acting on the model.
      // The external forces are stored in each vertex and consists of a downward gravity.
      // If a vertex is being dragged by the user, its velocity vector is added as an external force.
      real_type damping = 0.1;
      simulation_method::particle_iterator begin = m_simulator.particle_begin();
      simulation_method::particle_iterator end   = m_simulator.particle_end();
      simulation_method::particle_iterator particle;
      for (particle=begin;particle!=end; ++particle)
      {
        particle->f_ext() = vector3_type(0.0, -(particle->mass() * m_gravity), 0.0);
        vector3_type dir = unit(particle->v());
        particle->f_ext() -= damping*(particle->v()*particle->v())*dir;
      }
      if(m_pull_me)
      {
        m_pull_me->f_ext() += m_pull_force;
      }
    }
    m_simulator.run(dt);
  }

  void do_shutdown(){}

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
