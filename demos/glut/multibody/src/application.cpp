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

#include "data.h"
#include "scenes/setup_cow.h"
#include "scenes/setup_box.h"
#include "scenes/setup_cow_stack.h"
#include "scenes/setup_falling_cows.h"
#include "scenes/setup_silo.h"
#include "scenes/setup_diku.h"
#include "scenes/setup_silo_tower.h"
#include "scenes/setup_roof_tower.h"
#include "scenes/setup_domino_spiral.h"
#include "scenes/setup_domino_circle.h"
#include "scenes/setup_pyramid.h"
#include "scenes/setup_large_mass.h"
#include "scenes/setup_house.h"
#include "scenes/setup_card.h"
#include "scenes/setup_revolute_joint.h"
#include "scenes/setup_slider_joint.h"
#include "scenes/setup_ball_joint.h"
#include "scenes/setup_universal_joint.h"
#include "scenes/setup_wheel_joint.h"
#include "scenes/setup_vehicle.h"
#include "scenes/setup_box_stack.h"
#include "scenes/setup_jamm1.h"
#include "scenes/setup_jamm2.h"
#include "scenes/setup_ragdoll.h"


#include <OpenTissue/utility/utility_timer.h>

class Application : public OpenTissue::glut::PerspectiveViewApplication
{
protected:

  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.

  Data m_data;
  Vehicle m_vehicle;
  Ragdoll m_ragdoll;

  std::string   m_prefix;         ///< Prefix std::string used for generating images...
  size_type     m_framecount;     ///< frame counter...
  bool          m_recording_on;
  bool          m_wireframe_on;
  bool          m_simulation_on;
  bool          m_statistics_on;
  size_type     m_max_frames;
  size_type     m_inbetween;      ///< frames inbetween a generated key frame
  real_type     m_running_time;
  real_type     m_timestep;

  vector_type  m_timings;
  vector_type  m_contacts;
  vector_type  m_iterations;
  vector_type  m_accuracy;
  matrix_type  m_energy_kin;
  matrix_type  m_energy_pot;
  size_type m_cur;

protected:

  void reset_timestep()
  {
    m_timestep = 0.01;
    m_cur = 0;
    m_framecount = 0;
    m_max_frames = static_cast<size_type>( std::floor( m_running_time / m_timestep ) );
    m_inbetween =  static_cast<size_type>( std::floor( 0.1 / m_timestep ) );
  }

  void increment_timestep()
  {
    m_timestep *= 10.0;
    m_cur = 0;
    m_framecount = 0;
    m_max_frames = static_cast<size_type>( std::floor( m_running_time / m_timestep ) );
    m_inbetween =  static_cast<size_type>( std::floor( 0.1 / m_timestep ) );
  }

  void decrement_timestep()
  {
    m_timestep /= 10.0;
    m_cur = 0;
    m_framecount = 0;
    m_max_frames = static_cast<size_type>( std::floor( m_running_time / m_timestep ) );
    m_inbetween =  static_cast<size_type>( std::floor( 0.1 / m_timestep ) );
  }

public:

  Application() { }

public:

  char const * do_get_title() const { return "Multibody Demo Application"; }

  void do_display()
  {
    if(m_simulation_on || m_recording_on)
    {
      // Make sure wheels do not break their axes!
      if(m_data.m_wheel.get_wheel_body())
      {
        m_data.m_wheel.get_wheel_body()->set_finite_rotation_axis(m_data.m_wheel.get_motor_axis_world());
      }
      //Make sure vehicle is ready
      m_vehicle.update();

      OpenTissue::utility::Timer<real_type> watch;
      watch.start();
      m_data.m_simulator.run(m_timestep);
      watch.stop();
      if(!m_statistics_on)
      {
        std::cout << "time = " << watch() << " secs.";
        std::cout << "\t|C| = " << OpenTissue::mbd::compute_contact_count(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end());
        real_type min_dist;
        real_type max_dist;
        OpenTissue::mbd::compute_min_max_distance(m_data.m_configuration,min_dist,max_dist);
        std::cout << "\tmin = " << min_dist << "\tmax = " << max_dist << std::endl;
        std::cout << "\t accuracy = " << m_data.m_simulator.get_stepper()->get_solver()->get_accuracy() << " in " << m_data.m_simulator.get_stepper()->get_solver()->get_iteration() << " iterations" << std::endl;
      }
      if(m_statistics_on)
      {
        if(m_cur==0)
        {
          std::cout << "allocating for stats :  " << m_max_frames << std::endl;
          m_timings.resize(m_max_frames,false);
          m_contacts.resize(m_max_frames,false);
          m_iterations.resize(m_max_frames,false);
          m_accuracy.resize(m_max_frames,false);
          m_energy_kin.resize(m_data.m_configuration.size_bodies(),m_max_frames,false);
          m_energy_pot.resize(m_data.m_configuration.size_bodies(),m_max_frames,false);
        }

        if(m_cur<m_max_frames)
        {
          m_iterations(m_cur) = m_data.m_simulator.get_stepper()->get_solver()->get_iteration();
          m_accuracy(m_cur) = m_data.m_simulator.get_stepper()->get_solver()->get_accuracy();
          m_timings(m_cur) = m_data.m_simulator.time();
          m_contacts(m_cur) = OpenTissue::mbd::compute_contact_count(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end());
          size_type i = 0;
          for(configuration_type::body_iterator body =  m_data.m_configuration.body_begin();body!= m_data.m_configuration.body_end();++body)
          {
            m_energy_kin(i,m_cur) = body->compute_kinetic_energy();
            vector3_type p;
            body->get_position(p);
            m_energy_pot(i,m_cur) = body->get_mass()*9.81*p(2);
            ++i;
          }
          std::cout << "frame = " << m_cur << std::endl;
        }
        else if(m_cur==m_max_frames)
        {
          using namespace OpenTissue::math::big;


          std::cout << "writting statitics to file..." << std::endl;
          std::string filename = m_prefix + ".m";
          std::ofstream m_file(filename.c_str(),std::ios::out | std::ios::app);
          static int tmp = 0;

          m_file << "I"<< tmp <<" = " << m_iterations   << ";" << std::endl;
          m_file << "A"<< tmp <<" = " << m_accuracy     << ";" << std::endl;
          m_file << "T"<< tmp <<" = " << m_timings     << ";" << std::endl;
          m_file << "C"<< tmp <<" = " << m_contacts    << ";" << std::endl;
          m_file << "K"<< tmp <<" = " << m_energy_kin  << ";" <<std::endl;
          m_file << "P"<< tmp <<" = " << m_energy_pot  << ";" <<std::endl;
          ++tmp;
          m_file.flush();
          m_file.close();
        }

        ++m_cur;
      }
    }

    if(m_wireframe_on)
      for_each(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end(),OpenTissue::mbd::DrawBodyFunctor<true>());
    else
      for_each(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end(),OpenTissue::mbd::DrawBodyFunctor<false>());
    OpenTissue::mbd::draw_contacts(m_data.m_configuration);
    //OpenTissue::mbd::draw_penetrations(m_data.m_configuration);

    std::for_each( m_data.m_configuration.joint_begin(), m_data.m_configuration.joint_end(), OpenTissue::mbd::DrawJointFunctor() );

    if(m_recording_on)
    {
      std::string filename = m_prefix + ".mel";
      std::ofstream mel_file;
      if(m_framecount==0)
        mel_file.open(filename.c_str(),std::ios::out);
      else
        mel_file.open(filename.c_str(),std::ios::out | std::ios::app);
      if(!mel_file.is_open())
        std::cout << "error could not open file = " << filename.c_str() << std::endl;
      if(m_framecount==0)
        mel_file << OpenTissue::mbd::mel::geometry_string(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end())  << std::endl;
      if((m_framecount%m_inbetween)==0)
        mel_file << OpenTissue::mbd::mel::keyframe_string(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end(),m_data.m_simulator.time())  << std::endl;
      if(m_framecount==m_max_frames)
        mel_file << OpenTissue::mbd::mel::euler_filter_string(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end())  << std::endl;
      mel_file.flush();
      mel_file.close();
      std::cout << "frame = " << m_framecount << " time = " << m_data.m_simulator.time() << std::endl;
      ++m_framecount;
      if(m_framecount>m_max_frames)
        m_recording_on = false;
    }
  }

  void do_action(unsigned char choice)
  {
    // Toggle state
    m_action[choice] = ! m_action[choice];
    std::cout << "choice: " << choice << " = " << (int)choice << " value "<< m_action[choice] << std::endl;
    switch(choice)
    {
    case '1':
      m_prefix = "falling_cows";
      setup_falling_cows(m_data);
      reset_timestep();
      break;
    case '2':
      m_prefix = "cow_stack";
      setup_cow_stack(m_data);
      reset_timestep();
      break;
    case '3':
      m_prefix = "cow";
      setup_cow(m_data);
      reset_timestep();
      break;
    case '4':
      m_prefix = "box";
      setup_box(m_data);
      reset_timestep();
      break;
    case '5':
      m_prefix = "silo";
      setup_silo(m_data);
      reset_timestep();
      break;
    case '6':
      m_prefix = "diku";
      setup_diku(m_data);
      reset_timestep();
      break;
    case '7':
      m_prefix = "roof";
      setup_roof_tower(m_data);
      reset_timestep();
      break;
    case '8':
      m_prefix = "silo2";
      setup_silo_tower(m_data);
      reset_timestep();
      break;
    case '#':
      m_prefix = "domino_spiral";
      setup_domino_spiral(m_data);
      reset_timestep();
      break;
    case '¤':
      m_prefix = "domino_circle";
      setup_domino_circle(m_data);
      reset_timestep();
      break;
    case '%':
      m_prefix = "pyramid";
      setup_pyramid(m_data);
      reset_timestep();
      break;
    case '&':
      m_prefix = "large_mass";
      setup_large_mass(m_data);
      reset_timestep();
      break;
    case '/':
      m_prefix = "house";
      setup_house(m_data);
      reset_timestep();
      break;
    case '(':
      m_prefix = "card";
      setup_card(m_data);
      reset_timestep();
      break;
    case ')':
      m_prefix = "revolute";
      setup_revolute_joint(m_data);
      reset_timestep();
      break;
    case '=':
      m_prefix = "slider";
      setup_slider_joint(m_data);
      reset_timestep();
      break;
    case 'a':
      m_prefix = "ball";
      setup_ball_joint(m_data);
      reset_timestep();
      break;
    case 'b':
      m_prefix = "universal";
      setup_universal_joint(m_data);
      reset_timestep();
      break;
    case 'c':
      m_prefix = "wheel";
      setup_wheel_joint(m_data);
      reset_timestep();
      break;
    case 'd':
      m_prefix = "wheel";
      m_vehicle.setup(m_data);
      reset_timestep();
      break;
    case 'e':
      m_prefix = "box_stack";
      setup_box_stack(m_data);
      reset_timestep();
      break;
    case 'f':
      m_prefix = "penetrating_box_stack";
      setup_box_stack(m_data, 0.9);
      reset_timestep();
      break;
    case 'g':
      m_prefix = "jamm1";
      setup_jamm1(m_data);
      reset_timestep();
      break;
    case 'h':
      m_prefix = "jamm2";
      setup_jamm2(m_data);
      reset_timestep();
      break;
    case 'i':
      m_prefix = "ragdoll";
      m_ragdoll.setup(m_data);
      reset_timestep();
      break;
    case 'j':
      m_prefix = "pain";
      m_ragdoll.torture1(m_data);
      reset_timestep();
      break;
    case 'k':
      m_prefix = "agony";
      m_ragdoll.torture2(m_data);
      reset_timestep();
      break;
    case 'A':
      m_recording_on = true;
      m_framecount = 0;
      break;
    case 'D':    m_statistics_on = ! m_statistics_on;      break;
    case 'R':    m_simulation_on = !m_simulation_on;    break;
    case 'S':
      m_data.m_simulator.run(m_timestep);
      std::cout << "|C| = " << OpenTissue::mbd::compute_contact_count(m_data.m_configuration.body_begin(),m_data.m_configuration.body_end());
      real_type min_dist;
      real_type max_dist;
      OpenTissue::mbd::compute_min_max_distance(m_data.m_configuration,min_dist,max_dist);
      std::cout << "\tmin = " << min_dist << "\tmax = " << max_dist << std::endl;
      break;
    case 'W': m_wireframe_on = !m_wireframe_on;            break;
    case '+': increment_timestep();                        break;
    case '-': decrement_timestep();                        break;
    case 'r': reset_timestep();                            break;
    default: break;
    };
    std::cout << "#bodies         = " << m_data.m_configuration.size_bodies() << std::endl;
    std::cout << "Dump statistics = " << m_statistics_on                      << std::endl;
    std::cout << "MEL recording   = " << m_recording_on                       << std::endl;
    std::cout << "Run simulation  = " << m_simulation_on                      << std::endl;
    std::cout << "Timestep        = "
      << m_timestep
      << " max frames = "
      << m_max_frames
      << " inbetween = "
      << m_inbetween
      << std::endl;
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu(menu);
    glutAddMenuEntry("falling cows      [1]", '1');
    glutAddMenuEntry("cow stack         [2]", '2');
    glutAddMenuEntry("cow               [3]", '3');
    glutAddMenuEntry("box               [4]", '4');
    glutAddMenuEntry("silo               [5]", '5');
    glutAddMenuEntry("diku               [6]", '6');
    glutAddMenuEntry("roof               [7]", '7');
    glutAddMenuEntry("silo2              [8]", '8');
    glutAddMenuEntry("domino spiral      [#]", '#');
    glutAddMenuEntry("domino circel      [¤]", '¤');
    glutAddMenuEntry("pyramid            [%]", '%');
    glutAddMenuEntry("large mass         [&]", '&');
    glutAddMenuEntry("house              [/]", '/');
    glutAddMenuEntry("cards              [(]", '(');
    glutAddMenuEntry("revolute           [)]", ')');
    glutAddMenuEntry("slider             [=]", '=');
    glutAddMenuEntry("ball               [a]", 'a');
    glutAddMenuEntry("universal          [b]", 'b');
    glutAddMenuEntry("wheel              [c]", 'c');
    glutAddMenuEntry("vehicle            [d]", 'd');
    glutAddMenuEntry("box stack          [e]", 'e');
    glutAddMenuEntry("penetrating box stack [f]", 'f');
    glutAddMenuEntry("jamm 1             [g]", 'g');
    glutAddMenuEntry("jamm 2             [h]", 'h');
    glutAddMenuEntry("ragdoll            [i]", 'i');
    glutAddMenuEntry("pain               [j]", 'j');
    glutAddMenuEntry("agony              [k]", 'k');

    glutAddMenuEntry("MEL recording     [A]", 'A');
    glutAddMenuEntry("Dump statistics   [D]", 'D');
    glutAddMenuEntry("Run simulation    [R]", 'R');
    glutAddMenuEntry("Single Step       [S]", 'S');
    glutAddMenuEntry("Wireframe         [w]", 'W');
    glutAddMenuEntry("Decrease timestep [-]", '+');
    glutAddMenuEntry("Increase timestep [+]", '-');
    glutAddMenuEntry("Reset timestep    [r]", 'r');
    glutSetMenu(main_menu);
    glutAddSubMenu("multibody", controls);
  }

  void do_init()
  {
    m_wireframe_on  = false;
    m_simulation_on = false;
    m_statistics_on = false;
    m_recording_on  = false;

    OpenTissue::mbd::setup_default_geometry_dispatcher(m_data.m_simulator);

    m_running_time = 30.0;
    reset_timestep();
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
