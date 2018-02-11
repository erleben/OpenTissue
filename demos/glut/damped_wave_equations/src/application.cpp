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

#include <OpenTissue/dynamics/swe/swe_damped_wave_equation.h>
#include <OpenTissue/utility/utility_fps_counter.h>
#include <OpenTissue/utility/utility_timer.h>
#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/math/math_quaternion.h>
#include <OpenTissue/core/math/math_coordsys.h>
#include <OpenTissue/utility/gl/gl_util.h>

#include <string>



class Application : public OpenTissue::glut::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double,int>       math_types;
  typedef math_types::real_type                              real_type;
  typedef math_types::index_type                             index_type;
  typedef math_types::vector3_type                           vector3_type;
  typedef math_types::matrix3x3_type                         matrix3x3_type;

  typedef OpenTissue::swe::DampedWaveEquations<math_types>   damped_waves;
  typedef damped_waves::dwe_particle                         dwe_particle;

protected:

  damped_waves                         m_dwe;  ///< The main damped wave equation system
  OpenTissue::utility::FPSCounter<real_type>    m_fps;
  OpenTissue::utility::Timer<real_type>  m_sync;
  real_type                            m_timestep;      ///< Simulation timestep

  OpenTissue::gl::Camera<math_types>   m_camera;
  bool                                 m_zoom_mode;
  bool                                 m_pan_mode;
  bool                                 m_trackball_mode;
  real_type                            m_begin_x;
  real_type                            m_begin_y;
  real_type                            m_drop_percentage;
  unsigned int                         m_drop_rate;
  unsigned int                         m_cur_rate;

  enum {CLEAN          = 0x00,
    DRAW_GRID      = 0x01,
    DRAW_TRIANGLES = 0x02,
    DRAW_PARS      = 0x04,
    DRAW_NORMALS   = 0x08,
    AUTOMATION     = 0x10,
    SIMULATION     = 0x20,
    NOP            = 0x40,  // I miss the good ol' C64 days ;)
    INITIALIZED    = 0x80};
  unsigned char                        m_status;

protected:

  void flip_status(unsigned char bit) {if (m_status&bit) m_status&=~bit; else m_status|=bit;}

  void drop(real_type const& height)
  {
    index_type i = rand()%(m_dwe.columns()-1);
    index_type j = rand()%(m_dwe.rows()-1);
    m_dwe.particle(i,j).height() -= height;
    m_dwe.particle(i+1,j).height() -= height;
    m_dwe.particle(i,j+1).height() -= height;
    m_dwe.particle(i+1,j+1).height() -= height;
  }

  void wave(real_type const& height)
  {
    index_type j = m_dwe.rows()-11;
    index_type i = m_dwe.columns()/6;
    for (index_type n = 0; n < 2*m_dwe.columns()/3; ++n) {
      m_dwe.particle(i+n,j+0).height() -= height/6;
      m_dwe.particle(i+n,j+1).height() -= height/5;
      m_dwe.particle(i+n,j+2).height() -= height/4;
      m_dwe.particle(i+n,j+3).height() -= height/3;
      m_dwe.particle(i+n,j+4).height() -= height/2;
      m_dwe.particle(i+n,j+5).height() -= height;
      m_dwe.particle(i+n,j+6).height() -= height/2;
      m_dwe.particle(i+n,j+7).height() -= height/3;
      m_dwe.particle(i+n,j+8).height() -= height/4;
      m_dwe.particle(i+n,j+9).height() -= height/5;
      m_dwe.particle(i+n,j+10).height() -= height/6;
    }
  }


  /**
  * Put text onto screen (OSD)
  *
  *
  * 2007-05-20 kenny: what is wrong with OpenTissue::gl::DrawString???
  */
  void putText(real_type const& x, real_type const& y, real_type const& r, real_type const& g, real_type const& b, std::string const& text) const
  {
    // setup orthogonal projection to use 2D text
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glViewport(0, 0, this->width(), this->height());
    glOrtho(0, this->width(), 0,  this->height(), 0, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_LIGHTING);

    glColor4d(r, g, b, 1);
    glRasterPos2i( static_cast<int>( x ), static_cast<int>( y ) );
    for (const char *c = text.c_str(); *c != '\0'; ++c)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);//GLUT_BITMAP_8_BY_13
    glColor4d(r+.5, g+.5, b+.5, 1);
    glRasterPos2i( static_cast<int>( x+1 ), static_cast<int>( y-1 ) );
    for (const char *c = text.c_str(); *c != '\0'; ++c)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);//GLUT_BITMAP_8_BY_13

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
  }


public:

  Application(){}

public:

  char const * do_get_title() const { return "Damped Wave Equations Demo Application"; }

  void do_display()
  {
    const real_type d = 0.1;
    const vector3_type D(-d*0.5*m_dwe.columns(),0,-d*0.5*m_dwe.rows());
    if (m_status & DRAW_TRIANGLES) {
      glEnable(GL_COLOR_MATERIAL);
      glEnable(GL_LIGHTING);
      glColor3f(0.1, 0.1, 0.7);
      glBegin(GL_TRIANGLES);
      for (index_type j = 1; j < m_dwe.rows(); ++j) for (index_type i = 1; i < m_dwe.columns(); ++i) {
        dwe_particle const& p0 = m_dwe.particle(i-1,j-1);
        dwe_particle const& p1 = m_dwe.particle(i,j-1);
        dwe_particle const& p2 = m_dwe.particle(i,j);
        dwe_particle const& p3 = m_dwe.particle(i-1,j);
        const vector3_type n0 = m_dwe.normal(i-1,j-1);
        const vector3_type n1 = m_dwe.normal(i,j-1);
        const vector3_type n2 = m_dwe.normal(i,j);
        const vector3_type n3 = m_dwe.normal(i-1,j);
        glNormal3d(n0(0), n0(1), n0(2));
        glVertex3d(D(0)+d*(i-1), D(1)+p0.height(), D(2)+d*(j-1));
        glNormal3d(n1(0), n1(1), n1(2));
        glVertex3d(D(0)+d*i, D(1)+p1.height(), D(2)+d*(j-1));
        glNormal3d(n2(0), n2(1), n2(2));
        glVertex3d(D(0)+d*i, D(1)+p2.height(), D(2)+d*j);
        glNormal3d(n2(0), n2(1), n2(2));
        glVertex3d(D(0)+d*i, D(1)+p2.height(), D(2)+d*j);
        glNormal3d(n3(0), n3(1), n3(2));
        glVertex3d(D(0)+d*(i-1), D(1)+p3.height(), D(2)+d*j);
        glNormal3d(n0(0), n0(1), n0(2));
        glVertex3d(D(0)+d*(i-1), D(1)+p0.height(), D(2)+d*(j-1));
      }
      glEnd();
    }
    if (m_status & DRAW_GRID) {
      glDisable(GL_COLOR_MATERIAL);
      glDisable(GL_LIGHTING);
      glColor3f(0.1, 0.1, 0.7);
      glBegin(GL_LINES);
      for (index_type j = 0; j < m_dwe.rows(); ++j) for (index_type i = 1; i < m_dwe.columns(); ++i) {
        dwe_particle const& p0 = m_dwe.particle(i-1,j);
        dwe_particle const& p1 = m_dwe.particle(i,j);
        glVertex3d(D(0)+d*(i-1), D(1)+p0.height(), D(2)+d*j);
        glVertex3d(D(0)+d*i, D(1)+p1.height(), D(2)+d*j);
      }
      for (index_type i = 0; i < m_dwe.columns(); ++i) for (index_type j = 1; j < m_dwe.rows(); ++j) {
        dwe_particle const& p0 = m_dwe.particle(i,j-1);
        dwe_particle const& p1 = m_dwe.particle(i,j);
        glVertex3d(D(0)+d*i, D(1)+p0.height(), D(2)+d*(j-1));
        glVertex3d(D(0)+d*i, D(1)+p1.height(), D(2)+d*j);
      }
      glEnd();
    }
    if (m_status & DRAW_PARS) {
      glEnable(GL_COLOR_MATERIAL);
      glEnable(GL_LIGHTING);
      GLUquadric* qobj = gluNewQuadric();
      gluQuadricDrawStyle( qobj, GLU_FILL );
      for (index_type j = 0; j < m_dwe.rows(); ++j) for (index_type i = 0; i < m_dwe.columns(); ++i) {
        dwe_particle const& par = m_dwe.particle(i,j);
        // draw particles
        glPushMatrix();
        glTranslated(D(0)+d*i, D(1)+par.height(), D(2)+d*j);
        if (par.fixed())     // draw fixed particles in dark red
          glColor3f(0.7, 0.0, 0.0);
        else  // the other particles in dark grey
          glColor3f(0.1, 0.1, 0.7);
        gluSphere(qobj, 0.05, 12, 12);
        glPopMatrix();
      }
    }
    if (m_status & DRAW_NORMALS) {
      glDisable(GL_COLOR_MATERIAL);
      glDisable(GL_LIGHTING);
      glColor3f(0.9, 0.9, 0.1);
      glBegin(GL_LINES);
      for (index_type j = 0; j < m_dwe.rows(); ++j) for (index_type i = 0; i < m_dwe.columns(); ++i) {
        dwe_particle const& par = m_dwe.particle(i,j);
        vector3_type n = 0.2*m_dwe.normal(i,j);
        glVertex3d(D(0)+d*i, D(1)+par.height(), D(2)+d*j);
        glVertex3d(D(0)+n(0)+d*i, D(1)+n(1)+par.height(), D(2)+n(2)+d*j);
      }
      glEnd();
    }

    m_fps.frame();
    std::stringstream str;
    str << "FPS: " << m_fps();
    putText(6,this->height()-16,0,0,0,str.str());
    if ((m_status&INITIALIZED)&&m_dwe.smooth())
      putText(6,this->height()-36,0,0,0,"- smooth");
  }

  void do_action(unsigned char choice)
  {
    switch(choice)
    {
    case 'R':
      flip_status(SIMULATION);
      break;
    case 's': //--- Single stepping
      m_dwe.simulate(m_timestep);
      break;
    case '1':
      m_dwe.init(128, 128);
      m_dwe.damping() = 0.15;
      m_dwe.speed() = 60;
      m_dwe.spacing() = 6;
      for (index_type j = 0; j < m_dwe.rows(); ++j) for (index_type i = 0; i < m_dwe.columns(); ++i)
        m_dwe.particle(i,j).fixed() = j==0 || j==m_dwe.rows()-1 || i==0 || i==m_dwe.columns()-1;
      m_drop_percentage = 0.25;
      m_drop_rate = 20;
      m_status |= INITIALIZED;
      break;
    case '2':
      m_dwe.init(128, 128);
      m_dwe.damping() = 0.5;
      m_dwe.speed() = 100;
      m_dwe.spacing() = 4;
      for (index_type j = 0; j < m_dwe.rows(); ++j) for (index_type i = 0; i < m_dwe.columns(); ++i)
        m_dwe.particle(i,j).fixed() = j==0 || j==m_dwe.rows()-1 || i==0 || i==m_dwe.columns()-1;
      m_drop_percentage = 1;
      m_drop_rate = 2;
      m_status |= INITIALIZED;
      break;
    case 'A':
      flip_status(AUTOMATION);
      break;
    case 'S':
      m_dwe.smooth() = !m_dwe.smooth();
      break;
    case 'G':
      flip_status(DRAW_GRID);
      break;
    case 'T':
      flip_status(DRAW_TRIANGLES);
      break;
    case 'P':
      flip_status(DRAW_PARS);
      break;
    case 'N':
      flip_status(DRAW_NORMALS);
      break;
    case 'd':
      drop(0.1);
      break;
    case 'w':
      wave(0.035);
      break;
    default:
      break;
    } // End Switch
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int controls = glutCreateMenu(menu);
    glutAddMenuEntry("Inititialize Wave Pool [128x128]  [1]", '1');
    glutAddMenuEntry("Inititialize Rain Pool [128x128]  [2]", '2');
    glutAddMenuEntry("Start/Stop Simulation  [R]", 'R');
    glutAddMenuEntry("Single Step Simulation  [s]", 's');
    glutAddMenuEntry("Create Drop (Random)  [d]", 'd');
    glutAddMenuEntry("Create Wave  [w]", 'w');
    glutAddMenuEntry("Toggle Automation  [A]", 'A');
    glutAddMenuEntry("Toggle Smooth Wave Solver  [S]", 'S');
    glutAddMenuEntry("Toggle Grid  [G]", 'G');
    glutAddMenuEntry("Toggle Triangles  [T]", 'T');
    glutAddMenuEntry("Toggle Particles  [P]", 'P');
    glutAddMenuEntry("Toggle Normals  [N]", 'N');
    glutSetMenu(main_menu);
    glutAddSubMenu("wave", controls);
  }

  void do_init()
  {
    this->camera().init(vector3_type(0,10,15),vector3_type(0,0,0),vector3_type(0,1,0));

    m_status = DRAW_TRIANGLES|AUTOMATION;
    m_timestep = 0.02;
    m_cur_rate = 0;
    m_sync.start();
  }

  void do_run()
  {
    if (!((m_status&SIMULATION)&&(m_status&INITIALIZED))) return;
    m_sync.stop();
    if (m_sync() >= m_timestep) {
      m_sync.start();
      m_dwe.simulate(m_timestep);
      if ((m_status&AUTOMATION) && ++m_cur_rate >= m_drop_rate) {
        const int drops = boost::numeric_cast<int>(m_drop_percentage*1000);
        if (rand()%1000 >= 1000-drops)
          drop(0.1);
        else
          wave(0.15);
        m_cur_rate = 0;
      }
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
