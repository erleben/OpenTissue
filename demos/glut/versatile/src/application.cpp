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

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/dynamics/versatile/versatile.h>

#include <OpenTissue/core/containers/t4mesh/util/t4mesh_block_generator.h>
#include <OpenTissue/core/containers/t4mesh/io/t4mesh_xml_read.h>

#include <OpenTissue/utility/utility_timer.h>
#include <OpenTissue/core/math/math_constants.h>




class Application 
  : public OpenTissue::glut::PerspectiveViewApplication
{
public:

  typedef OpenTissue::math::BasicMathTypes<double,size_t> math_types;

  typedef math_types::real_type                     real_type;
  typedef math_types::value_traits                  value_traits;
  typedef math_types::vector3_type                  vector3_type;
  typedef math_types::matrix3x3_type                matrix3x3_type;

  typedef OpenTissue::versatile::Types<math_types>  versatile_types;
  typedef versatile_types::mesh_type                mesh_type;

protected:

  struct world_point_container
  {
    world_point_container(mesh_type* mesh) : m_mesh(mesh) {}

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

    mesh_type * m_mesh;
  };


protected:

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.
  
  static unsigned char const A_KEY = 'a';
  static unsigned char const B_KEY = 'b';
  static unsigned char const D_KEY = 'd';
  static unsigned char const V_KEY = 'v';

protected:

  mesh_type                          m_objects[10];    ///< Ojects to be simulated.
  real_type                          m_gravity;        ///< Acceleration of gravity.
  versatile_types::simulator_type    m_simulator;         ///< The simulation method...

  real_type    m_k_D;   ///< Stiffness of distance constraints.
  real_type    m_k_A;   ///< Stiffness of area constraints.
  real_type    m_k_V;   ///< Stiffness of volume constraints.

public:

  Application()
    : m_gravity(9.81) 
    , m_k_D(100)
    , m_k_A(10)
    , m_k_V(40)
  { }

public:

  char const * do_get_title() const { return "Versatile Demo Application"; }

  void do_display()
  {
    if(m_action[B_KEY])
    {
      OpenTissue::gl::ColorPicker(0.5,0.0,0.5);
      OpenTissue::gl::DrawPointsT4Mesh(world_point_container(&m_objects[0]),m_objects[0],0.95,false);
      OpenTissue::gl::ColorPicker(0.5,0.0,0.0);
      OpenTissue::gl::DrawPointsT4Mesh(world_point_container(&m_objects[1]),m_objects[1],0.95,true);
    }
    else
    {
      for(unsigned int i=0;i<2;++i)
      {
        OpenTissue::gl::ColorPicker(0.5,0.0,0.5);
        OpenTissue::gl::DrawPointsT4Mesh(world_point_container(&m_objects[i]),m_objects[i],0.95,true);
        OpenTissue::gl::ColorPicker(0.5,0.0,0.0);
        OpenTissue::gl::DrawVersatilePenaltyForces( m_objects[i] );
        OpenTissue::gl::ColorPicker(0.0,0.5,0.0);
        OpenTissue::gl::DrawVersatileInternalForces( m_objects[i] );
        OpenTissue::gl::ColorPicker(0.0,0.0,0.5);
        OpenTissue::gl::DrawVersatileExternalForces( m_objects[i] );
      }
    }
  }

  void do_action(unsigned char choice)
  {
    using std::min;
    using std::max;

    // Toggle state
    m_action[choice] = ! m_action[choice];
    //cout << "choice: " << choice << " = " << (int)choice << " value "<< m_action[choice] << endl;
    switch(choice)
    {
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
        unsigned int I,J,K;
        real_type width  = .2;
        real_type height = .2;
        real_type depth  = .2;
        for(unsigned int i=0;i<2;++i)
        {
          world_point_container point_wrapper(&m_objects[i]);
          if(i==0)
          {
            I = 5;
            J = 5;
            K = 2;
          }
          else
          {
            I = 2;
            J = 2;
            K = 2;
          }
          OpenTissue::t4mesh::generate_blocks(I,J,K,width,height,depth,m_objects[i]);
          mesh_type::node_iterator begin = m_objects[i].node_begin();
          mesh_type::node_iterator end = m_objects[i].node_end();
          mesh_type::node_iterator node;
          if(i==0)
          {
            for ( node=begin; node != end; ++node)
            {
              node->m_coord += vector3_type(-.25,0,-.25);
              node->m_x0 = node->m_coord;
              node->m_mass = 1.0;
              //      if(node->m_coord(0) < 0.01)
              node->m_fixed = true;
            }
          }
          else
          {
            for ( node=begin; node != end; ++node)
            {
              node->m_coord += vector3_type(0,1,0);
              node->m_x0 = node->m_coord;
              node->m_mass = 1.0;
            }
          }
          m_objects[i].initialize();
          m_objects[i].set_distance_coefficients(m_k_D,0.01);
          m_objects[i].set_area_coefficients(m_k_A,0);
          m_objects[i].set_volume_coefficients(m_k_V,0);
          real_type c_yield = 0.1;  // set to infinity to turn of plasticity
          real_type c_creep = 0.1;
          real_type c_max = 1.0;
          m_objects[i].set_plasticity(c_yield,c_creep,c_max);
          m_simulator.add(m_objects[i]);
        }
      }
      break;
    case 'b':
      {
        m_simulator.clear();
        unsigned int I,J,K;
        real_type width  = .2;
        real_type height = .2;
        real_type depth  = .2;

        world_point_container point_wrapper0(&m_objects[0]);
        I = 10;
        J = 3;
        K = 3;
        OpenTissue::t4mesh::generate_blocks(I,J,K,width,height,depth,m_objects[0]);

        world_point_container point_wrapper1(&m_objects[1]);
        OpenTissue::t4mesh::generate_blocks(I,J,K,width,height,depth,m_objects[1]);

        mesh_type::node_iterator begin = m_objects[0].node_begin();
        mesh_type::node_iterator end = m_objects[0].node_end();
        mesh_type::node_iterator node;
        for ( node=begin; node != end; ++node)
        {
          if(node->m_coord(0) < .1)
            node->m_fixed = true;
          node->m_x0 = node->m_coord;
          node->m_mass = 1.0;
        }
        m_objects[0].initialize();

        m_k_D = 200;
        m_k_A = 0;
        m_k_V = 40;

        m_objects[0].set_distance_coefficients(m_k_D,0.001);
        m_objects[0].set_area_coefficients(m_k_A,0);
        m_objects[0].set_volume_coefficients(m_k_V,0);
        real_type c_yield = value_traits::infinity(); //--- These plasticity settings means that plasticity is turned off
        real_type c_creep = value_traits::zero();
        real_type c_max   = value_traits::zero();
        m_objects[0].set_plasticity(c_yield,c_creep,c_max);
        m_simulator.add(m_objects[0]);
      }
      break;
    case '+':
      if(m_action[D_KEY])
        m_k_D = min(m_k_D+0.1 , 100.0);
      if(m_action[A_KEY])
        m_k_A = min(m_k_A+0.1 , 100.0);
      if(m_action[V_KEY])
        m_k_V = min(m_k_V+0.1 , 100.0);
      for(unsigned int i=0;i<2;++i)
      {
        m_objects[i].set_distance_coefficients(m_k_D,0.001);
        m_objects[i].set_area_coefficients(m_k_A,0);
        m_objects[i].set_volume_coefficients(m_k_V,0);
      }
      std::cout << "k_D = " << m_k_D << " k_A = " << m_k_A << " k_V = " << m_k_V << std::endl;
      break;
    case '-':
      if(m_action[D_KEY])
        m_k_D = max(m_k_D-0.1,0.0);
      if(m_action[A_KEY])
        m_k_A = max(m_k_A-0.1,0.0);
      if(m_action[V_KEY])
        m_k_V = max(m_k_V-0.1,0.0);
      for(unsigned int i=0;i<2;++i)
      {
        m_objects[i].set_distance_coefficients(m_k_D,0.001);
        m_objects[i].set_area_coefficients(m_k_A,0);
        m_objects[i].set_volume_coefficients(m_k_V,0);
      }
      std::cout << "k_D = " << m_k_D << " k_A = " << m_k_A << " k_V = " << m_k_V << std::endl;
      break;
    case '1':
      m_k_D = 30;
      m_k_A = 0;
      m_k_V = 40;
      for(unsigned int i=0;i<2;++i)
      {
        m_objects[i].set_distance_coefficients(m_k_D,0.001);
        m_objects[i].set_area_coefficients(m_k_A,0);
        m_objects[i].set_volume_coefficients(m_k_V,0);
      }
      std::cout << "k_D = " << m_k_D << " k_A = " << m_k_A << " k_V = " << m_k_V << std::endl;
      break;
    case '2':
      m_k_D = 100;
      m_k_A = 0;
      m_k_V = 1;
      for(unsigned int i=0;i<2;++i)
      {
        m_objects[i].set_distance_coefficients(m_k_D,0.001);
        m_objects[i].set_area_coefficients(m_k_A,0);
        m_objects[i].set_volume_coefficients(m_k_V,0);
      }
      std::cout << "k_D = " << m_k_D << " k_A = " << m_k_A << " k_V = " << m_k_V << std::endl;
      break;
    case '3':
      m_k_D = 20;
      m_k_A = 0;
      m_k_V = 10;
      for(unsigned int i=0;i<2;++i)
      {
        m_objects[i].set_distance_coefficients(m_k_D,0.001);
        m_objects[i].set_area_coefficients(m_k_A,0);
        m_objects[i].set_volume_coefficients(m_k_V,0);
      }
      std::cout << "k_D = " << m_k_D << " k_A = " << m_k_A << " k_V = " << m_k_V << std::endl;
      break;;
    case 'a':  std::cout << "Toggle area stiffness = " << m_action[A_KEY] << std::endl; break;
    case 'd':  std::cout << "Toggle distance stiffness = " << m_action[D_KEY] << std::endl; break;
    case 'v':  std::cout << "Toggle volume stiffness = " << m_action[V_KEY] << std::endl; break;
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
    glutAddMenuEntry("Preset 1 [1]", '1');
    glutAddMenuEntry("Preset 2 [2]", '2');
    glutAddMenuEntry("Preset 3 [3]", '3');
    glutAddMenuEntry("Toggle Area Stiffness [a]", 'a');
    glutAddMenuEntry("Toggle Distance Stiffness [d]", 'd');
    glutAddMenuEntry("Toggle Volume Stiffness [v]", 'v');
    glutAddMenuEntry("Increase Stiffness [+]", '+');
    glutAddMenuEntry("Decrease Stiffness [-]", '-');
    glutSetMenu(main_menu);
    glutAddSubMenu("versatile", controls);
  }

  void do_init()
  {
    this->camera().move(95);
  }

  void do_run()
  {
    real_type dT = 0.01;
    real_type fraction = 0.05;

    {
      // Calculate external forces acting on the model.
      // The external forces are stored in each vertex and consists of a downward gravity.
      // If a vertex is being dragged by the user, its velocity vector is added as an external force.
      real_type damping = 2.;

      for(unsigned int i=0;i<2;++i)
      {
        mesh_type::node_iterator node = m_objects[i].node_begin();
        for ( ; node != m_objects[i].node_end(); ++node)
        {
          node->m_f_ext = vector3_type(0.0, -(node->m_mass * m_gravity), 0.0);
          vector3_type dir = OpenTissue::math::unit(node->m_v);
          node->m_f_ext -= damping*(node->m_v*node->m_v)*dir;
        }
      }
    }

    m_simulator.run(dT,fraction);
    //std::cout << "E = " << m_simulator.compute_internal_energy() << std::endl;
  }

  void do_shutdown(){}

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
