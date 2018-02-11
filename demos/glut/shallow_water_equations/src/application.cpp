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

#include <OpenTissue/dynamics/swe/swe_shallow_water_equation.h>


class Application : public OpenTissue::glut::PerspectiveViewApplication
{
protected:

  OpenTissue::swe::ShallowWaterEquations<> m_water;
  bool m_action[ 256 ];          ///< Boolean array used to keep track of the user actions/selections.
  float        m_timestep;       ///< Simulation timestep

  static unsigned char const ONE_KEY   = '1';
  static unsigned char const TWO_KEY   = '2';
  static unsigned char const THREE_KEY = '3';
  static unsigned char const FOUR_KEY  = '4';
  static unsigned char const FIVE_KEY  = '5';
  static unsigned char const D_KEY     = 'd';
  
public:

  Application(){}

public:

  char const * do_get_title() const { return "Shallow Water Equations Demo Application"; }

  void do_display()
  {
    if(m_action[ONE_KEY] || m_action[TWO_KEY] || m_action[THREE_KEY] || m_action[FOUR_KEY]|| m_action[FIVE_KEY])
    {
      m_water.draw(m_action[D_KEY]);
    }
  }

  void do_action(unsigned char choice)
  {
    m_action[choice] = ! m_action[choice];

    int M = 90;
    int N = 90;

    switch(choice)
    {
    case 'd':break;
    case '1':
      {
        m_water.init(M,N,16,16);
        for(int i=0;i<M;++i)
          for(int j=0;j<N;++j)
          {
            m_water.setShore(i,j,false);
            m_water.setSeaBottom(i,j,0);
            if(M<2*i && N<2*j)
              m_water.setSeaHeight(i,j,0);
            else
              m_water.setSeaHeight(i,j,2);
            m_water.setSeaVelocity(i,j,0,0);
          }
          m_action[ONE_KEY] = true;
          m_action[TWO_KEY] = false;
          m_action[THREE_KEY] = false;
          m_action[FOUR_KEY] = false;
          m_action[FIVE_KEY] = false;
      }
      break;
    case '2':
      {
        m_water.init(M,N,16,16);
        for(int i=0;i<M;++i)
          for(int j=0;j<N;++j)
          {
            m_water.setShore(i,j,false);
            m_water.setSeaBottom(i,j,0);
            if(M<3*i && 3*i<2*M && N<3*j && 3*j<2*N)
              m_water.setSeaHeight(i,j,0);
            else
              m_water.setSeaHeight(i,j,2);
            m_water.setSeaVelocity(i,j,0,0);
          }
          m_action[ONE_KEY] = false;
          m_action[TWO_KEY] = true;
          m_action[THREE_KEY] = false;
          m_action[FOUR_KEY] = false;
          m_action[FIVE_KEY] = false;
      }
      break;
    case '3':
      {
        m_water.init(M,N,16,16);
        for(int i=0;i<M;++i)
          for(int j=0;j<N;++j)
          {
            m_water.setShore(i,j,false);

            double b = (2./(M-1))*i;
            m_water.setSeaBottom(i,j,b);

            if(2*i<M)
              m_water.setSeaHeight(i,j,2);
            else
              m_water.setSeaHeight(i,j,0);
            m_water.setSeaVelocity(i,j,0,0);
          }
          m_action[ONE_KEY] = false;
          m_action[TWO_KEY] = false;
          m_action[THREE_KEY] = true;
          m_action[FOUR_KEY] = false;
          m_action[FIVE_KEY] = false;
      }
      break;
    case '4':
      {
        m_water.init(M,N,16,16);
        for(int i=0;i<M;++i)
          for(int j=0;j<N;++j)
          {
            m_water.setShore(i,j,false);
            if(2.5*i<M)
              m_water.setSeaHeight(i,j,2);
            else
              m_water.setSeaHeight(i,j,0);
            double b = (2./(M-1))*i;
            m_water.setSeaBottom(i,j,b);
            m_water.setSeaVelocity(i,j,0,0);
          }
          m_action[ONE_KEY] = false;
          m_action[TWO_KEY] = false;
          m_action[THREE_KEY] = false;
          m_action[FOUR_KEY] = true;
          m_action[FIVE_KEY] = false;
      }
      break;
    case '5':
      {
        m_water.init(M,N,16,16);
        for(int i=0;i<M;++i)
          for(int j=0;j<N;++j)
          {
            m_water.setSeaBottom(i,j,0);
            m_water.setSeaVelocity(i,j,0,0);


            if(M<3*i && 3*i<2*M && N<3*j && 3*j<2*N)
              m_water.setSeaBottom(i,j,1);

            if(M<2*i && N<2*j)
              m_water.setSeaHeight(i,j,1.1);
            else
              m_water.setSeaHeight(i,j,3);

            if(i<10 || j>80 ||  j<10 || i>80)
            {
              m_water.setShore(i,j,true);
            }
            else
              m_water.setShore(i,j,false);

            if(15<i && i<25 && 15<j && j<25)
              m_water.setShore(i,j,true);
          }
          m_action[ONE_KEY] = false;
          m_action[TWO_KEY] = false;
          m_action[THREE_KEY] = false;
          m_action[FOUR_KEY] = false;
          m_action[FIVE_KEY] = true;
      }
      break;
      break;
    case 's'://--- Single stepping
      run();
      break;
    default:
      std::cout << "You pressed " << choice << std::endl;
      break;
    };// End Switch
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu(menu);
    glutAddMenuEntry("shallow water config 1 [1]", '1');
    glutAddMenuEntry("shallow water config 2 [2]", '2');
    glutAddMenuEntry("shallow water config 3 [3]", '3');
    glutAddMenuEntry("shallow water config 4 [4]", '4');
    glutAddMenuEntry("shallow water config 5 [5]", '5');
    glutAddMenuEntry("Single Stepping        [s]", 's');

    glutSetMenu(main_menu);
    glutAddSubMenu("Water", controls);
  }

  void do_init()
  {
    m_timestep = 0.02;
  }

  void do_run()
  {
    if(m_action[ONE_KEY] || m_action[TWO_KEY] || m_action[THREE_KEY] || m_action[FOUR_KEY]|| m_action[FIVE_KEY])
    {
      // To increase visualization speed, we turn off statistics.
      m_water.run(m_timestep, false);
    }
  }

  void do_shutdown(){}

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
