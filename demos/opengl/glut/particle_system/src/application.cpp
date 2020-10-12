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

#include <OpenTissue/dynamics/psys/psys.h>
#include <OpenTissue/core/math/math_basic_types.h>


class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
public:

  typedef OpenTissue::math::BasicMathTypes<double,size_t>      math_types;
  typedef OpenTissue::psys::Types<math_types>            types;
  //typedef OpenTissue::psys::Types<double,OpenTissue::psys::EulerIntegrator> types;

  typedef math_types::real_type                           real_type;
  typedef math_types::vector3_type                        vector3_type;
  typedef math_types::matrix3x3_type                      matrix3x3_type;

protected:

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.
  
  static unsigned char const ONE_KEY   = '1';
  static unsigned char const TWO_KEY   = '2';
  static unsigned char const THREE_KEY = '3';
  static unsigned char const FOUR_KEY  = '4';
  static unsigned char const FIVE_KEY  = '5';
  static unsigned char const SIX_KEY   = '6';
  static unsigned char const SEVEN_KEY = '7';
  
  types::mass_spring_system_type           m_mass_spring;
  types::cloth_system_type                 m_cloth;
  types::surface_system_type               m_surface;

  types::sphere_type                       m_sphere;
  types::plane_type                        m_plane;
  types::sdf_geometry_type                 m_sdf_geometry;
  types::aabb_tree_type                    m_aabb_tree;

  types::gravity_type                      m_gravity;
  types::viscosity_type                    m_viscosity;

  types::grid_force_field_type::grid_type   m_force_grid;
  types::grid_force_field_type             m_force_field;
  types::pressure_soft_body_type           m_pressure;

  types::mesh_type                         m_mesh;

  types::box_constraint_type               m_box;
  types::pin_constraint_type               m_pin;

  real_type                                m_timestep;
  bool                                     m_simulate_on;  ///< Boolean flag used to toggle simulation on/off.

protected:

  void cleanup(  )
  {
    m_surface.clear();
    m_cloth.clear();
    m_mass_spring.clear();

    m_gravity.disconnect();
    m_viscosity.disconnect();
    m_pressure.disconnect();
    m_force_field.disconnect();

    m_pin.disconnect();
    m_box.disconnect();

    m_mesh.clear();
  }

  void cloth_setup( 
       bool create_sticks = true
    ,  bool create_springs = false
    , unsigned int rigidity = 2 
    )
  {
    cleanup();

    m_cloth.rigidty() = rigidity;
    m_cloth.init(1.75,1.75,20,20, create_sticks, create_springs);
    m_cloth.iterations() = 10;
    m_cloth.relaxation() = true;
    m_cloth.projection() = true;

    //--- Initialize geometries... ---------------------------------------------------
    m_plane.n() = vector3_type(0,0,1);
    m_plane.w() = -5;
    m_cloth.add_geometry( &m_plane );

    //--- Initialize forces ----------------------------------------------------------
    m_gravity.gravity() = 9.8;
    m_viscosity.viscosity() = 0.5;
    m_cloth.add_force(&m_gravity);
    m_cloth.add_force(&m_viscosity);
    //--- Initialize constraints -----------------------------------------------------

    m_box.aabb().min() = vector3_type(-3,-3,-6);
    m_box.aabb().max() = vector3_type( 3, 3, 6);
    m_cloth.add_constraint( &m_box );

    if(create_springs)
      m_timestep = 0.001;
    else
      m_timestep = 0.01;
  }

  void surface_setup( 
      std::string const & filename
    , bool create_sticks = true
    ,  bool create_springs = false
    , unsigned int rigidity = 3 
    )
  {
    cleanup();

    std::string data_path = opentissue_path;
    std::string meshfile = data_path + filename;
    OpenTissue::mesh::obj_read(meshfile,m_mesh);
    OpenTissue::mesh::make_unit( m_mesh );
    OpenTissue::mesh::translate(m_mesh,vector3_type( -0.5, -0.5, 5));

    m_surface.rigidty() = rigidity;
    m_surface.init(m_mesh, create_sticks, create_springs);
    m_surface.iterations() = 10;
    m_surface.relaxation() = true;
    m_surface.projection() = true;

    //--- Initialize geometries... ---------------------------------------------------
    m_sphere.radius(4);
    m_sphere.center( vector3_type(0,0,-8) );

    m_plane.n() = vector3_type(0,0,1);
    m_plane.w() = -5;

    m_surface.add_geometry( &m_sphere       );
    m_surface.add_geometry( &m_plane        );

    //--- Initialize forces ----------------------------------------------------------
    m_gravity.gravity() = 9.8;
    m_viscosity.viscosity() = 0.5;
    m_surface.add_force(&m_gravity);
    m_surface.add_force(&m_viscosity);
    //--- Initialize constraints -----------------------------------------------------

    m_box.aabb().min() = vector3_type(-3,-3,-6);
    m_box.aabb().max() = vector3_type( 3, 3, 6);
    m_surface.add_constraint( &m_box );

    if(create_springs)
      m_timestep = 0.001;
    else
      m_timestep = 0.01;
  }

public:

  Application(){}

public:

  char const * do_get_title() const { return "Particle System Demo Application"; }

  void do_display()
  {

    OpenTissue::gl::ColorPicker( 0.1, 0.4, 0.8 );
    OpenTissue::gl::DrawSphere( m_sphere, true );
    OpenTissue::gl::DrawPlane( m_plane, true );

    OpenTissue::gl::ColorPicker( 0.1, 0.8, 0.4 );

    if(!m_surface.coupling().empty())
      OpenTissue::gl::DrawMesh( m_surface.coupling().mesh() , GL_LINE_LOOP );

    if(!m_cloth.coupling().empty())
      OpenTissue::gl::DrawMesh( m_cloth.coupling().mesh() , GL_LINE_LOOP );

    OpenTissue::gl::ColorPicker( 0.8, 0.4, 0.1 );
    OpenTissue::gl::DrawAABB( m_box.aabb(), true );
    OpenTissue::gl::ColorPicker( 0.8, 0.4, 0.1 );
    OpenTissue::gl::DrawPoint( m_pin.pin_position() );

    OpenTissue::gl::ColorPicker( 0.8, 0.8, 0.1 );
    //OpenTissue::aabb_tree_debug_draw(m_aabb_tree);

    //OpenTissue::sdf::debug_draw_sampling( m_sdf_geometry );  
    //OpenTissue::sdf::debug_draw_bvh( m_sdf_geometry, 0);
    OpenTissue::gl::DrawMesh( m_sdf_geometry.m_mesh , GL_LINE_LOOP );

    OpenTissue::gl::ColorPicker( 0.8, 0.1, 0.8 );
    types::mass_spring_system_type::particle_iterator p = m_mass_spring.particle_begin();
    types::mass_spring_system_type::particle_iterator end = m_mass_spring.particle_end();
    for(;p!=end;++p)
    {
      OpenTissue::gl::DrawPoint(p->position());
    }

  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];
    switch ( choice )
    {
    case 't':       run();   break;
    case 's':    
      m_simulate_on = !m_simulate_on;      
      std::cout << " auto simulate = " << m_simulate_on << std::endl;
      break;
    case '1':
      {
        surface_setup("/demos/data/obj/box.obj");    

        m_action[ONE_KEY] = true;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '2':    
      {
        surface_setup("/demos/data/obj/box.obj");
        m_pin.init( &(*m_surface.particle_begin()) );
        m_surface.add_constraint( &m_pin );

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = true;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '3':    
      {
        surface_setup("/demos/data/obj/box.obj");

        m_surface.remove_force( &m_gravity );

        m_force_grid.create( vector3_type(-5,-5,-5), vector3_type(5,5,5), 32, 32, 32);
        OpenTissue::psys::compute_perlin_noise_force_field(m_force_grid );//--- just a compile test:-)
        OpenTissue::psys::compute_random_force_field(m_force_grid, 10.0 );

        m_force_field.init(  m_force_grid );
        m_surface.add_force(&m_force_field);

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = true;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '4':
      {
        surface_setup("/demos/data/obj/cylinder.obj", false, true, 1);
        m_pressure.set_initial_pressure( 5000.0 );
        m_pressure.init( m_surface.coupling() );
        m_surface.add_force(&m_pressure);

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = true;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '5':
      {
        cloth_setup();

        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/box.obj";
        OpenTissue::mesh::obj_read(meshfile,m_mesh);
        OpenTissue::mesh::make_unit( m_mesh );
        OpenTissue::mesh::translate(m_mesh,vector3_type( 0, 0, -5));

        OpenTissue::sdf::semiauto_init_geometry( m_mesh, 0.01, true, m_sdf_geometry);
        m_cloth.add_geometry( &m_sdf_geometry );

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = true;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '6':
      {
        cleanup();
        //--- Initialize geometries... ---------------------------------------------------
        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/propella.obj";
        OpenTissue::mesh::obj_read(meshfile,m_mesh);
        OpenTissue::mesh::make_unit( m_mesh );
        OpenTissue::mesh::uniform_scale(m_mesh, 2.0 );

        m_surface.rigidty() = 2;
        m_surface.init(m_mesh, true, false);
        m_surface.iterations() = 10;
        m_surface.relaxation() = true;
        m_surface.projection() = true;
        OpenTissue::aabb_tree::init( m_surface.coupling().mesh() , m_aabb_tree, m_surface.coupling() );

        m_mass_spring.add_geometry( &m_aabb_tree    );

        m_plane.n() = vector3_type(0,0,1);
        m_plane.w() = -5;
        m_mass_spring.add_geometry( &m_plane        );

        //--- Initialize constraints -----------------------------------------------------
        m_box.aabb().min() = vector3_type(-3,-3,-6);
        m_box.aabb().max() = vector3_type( 3, 3, 6);
        m_mass_spring.add_constraint( &m_box );

        m_timestep = 0.01;

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = true;
        m_action[SEVEN_KEY] = false;
      }
      break;
    case '7':
      {
        cleanup();
        //--- Initialize geometries... ---------------------------------------------------
        std::string data_path = opentissue_path;
        std::string meshfile = data_path + "/demos/data/obj/propella.obj";
        OpenTissue::mesh::obj_read(meshfile,m_mesh);
        OpenTissue::mesh::make_unit( m_mesh );
        OpenTissue::mesh::uniform_scale(m_mesh, 2.0 );

        m_surface.rigidty() = 2;
        m_surface.init(m_mesh, true, false);
        m_surface.iterations() = 10;
        m_surface.relaxation() = true;
        m_surface.projection() = true;
        OpenTissue::aabb_tree::init( m_surface.coupling().mesh() , m_aabb_tree, m_surface.coupling() );

        m_surface.add_geometry( &m_aabb_tree    );

        m_sphere.radius(4);
        m_sphere.center( vector3_type(0,0,-8) );
        m_surface.add_geometry( &m_sphere       );

        m_plane.n() = vector3_type(0,0,1);
        m_plane.w() = -5;
        m_surface.add_geometry( &m_plane        );

        m_gravity.gravity() = 9.8;
        m_viscosity.viscosity() = 0.5;
        m_surface.add_force(&m_gravity);
        m_surface.add_force(&m_viscosity);

        m_timestep = 0.01;

        m_action[ONE_KEY] = false;
        m_action[TWO_KEY] = false;
        m_action[THREE_KEY] = false;
        m_action[FOUR_KEY] = false;
        m_action[FIVE_KEY] = false;
        m_action[SIX_KEY] = false;
        m_action[SEVEN_KEY] = true;
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
    glutAddMenuEntry("Toggle auto simulate                  [s]", 's' );
    glutAddMenuEntry("Single step                           [t]", 't' );
    glutAddMenuEntry("Stick Box setup                       [1]", '1' );
    glutAddMenuEntry("Pin constraint                        [2]", '2' );
    glutAddMenuEntry("Random force field                    [3]", '3' );
    glutAddMenuEntry("Pressure forces                       [4]", '4' );
    glutAddMenuEntry("cloth vs. signed distance field test  [5]", '5' );
    glutAddMenuEntry("particle sys vs. particle sys         [6]", '6' );
    glutAddMenuEntry("particle self-collision               [7]", '7' );

    glutSetMenu( main_menu );
    glutAddSubMenu( "particle system", controls );
  }

  void do_init()
  {
    m_simulate_on = false;
  }

  void do_run()
  {
    if(m_simulate_on)
    {
      if(m_action[SIX_KEY])
      {
        if(m_mass_spring.particles_size()<50)
        {
          types::particle_type p;
          OpenTissue::math::random(p.position(),-1,1);
          p.position()(0) = -2;
          p.velocity() = vector3_type(1.0,0,0);
          p.old_position() = p.position() - p.velocity()*m_timestep;
          m_mass_spring.create_particle( p );      
        }
        m_mass_spring.run( m_timestep);
      }
      else
      {
        m_surface.run(m_timestep);
        m_cloth.run(m_timestep);
      }
    }
  }

  void do_shutdown(){}

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
