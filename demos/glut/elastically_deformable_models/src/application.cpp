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


#include <OpenTissue/dynamics/edm/edm.h>
//#include <OpenTissue/dynamics/edm/io/edm_system_xml_read.h> // completely broken :(
#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_torus.h>
#include <OpenTissue/core/geometry/geometry_capsule.h>
#include <OpenTissue/gpu/image/image.h>
#include <OpenTissue/gpu/image/io/image_read.h>
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/math/math_functions.h>
#include <OpenTissue/utility/utility_fps_counter.h>

#include <cstring>


template <typename M>
class EDMMaterial 
{
public:

  typedef M                                  math_types;
  typedef typename math_types::value_traits  value_traits;
  typedef typename math_types::real_type     real_type;

  enum {
      MATERIAL_NONE
    , MATERIAL_COLOR
    , MATERIAL_TEXTURE
  };

public:

  EDMMaterial()
    : m_type(MATERIAL_NONE)
    , m_r(value_traits::zero())
    , m_g(value_traits::zero())
    , m_b(value_traits::zero())
    , m_a(value_traits::one())
    , m_texture(GLuint(-1))
  {}

  ~EDMMaterial()
  {
    clear_material();
  }

public:

  int const & get_material_type() const
  {
    return m_type;
  }

  void  clear_material()
  {
    if (m_texture+GLuint(1) >= 0)
      glDeleteTextures(1, &m_texture);
    m_texture = GLuint(-1);
    m_type = MATERIAL_NONE;
  }

  bool  set_material(real_type const & red, real_type const & green, real_type const & blue, real_type const & alpha = value_traits::one())
  {
    m_r = OpenTissue::math::clamp_zero_one(red);
    m_g = OpenTissue::math::clamp_zero_one(green);
    m_b = OpenTissue::math::clamp_zero_one(blue);
    m_a = OpenTissue::math::clamp_zero_one(alpha);

    m_type = MATERIAL_COLOR;

    return true;
  }

  bool  set_material(std::string const & filename)
  {
    clear_material();

    OpenTissue::image::Image<unsigned char> bm;
    if (!OpenTissue::image::read(filename, bm, false)) {
      std::cerr << "\"" << filename << "\" is unknown or unsupported!" << std::endl;
      return false;
    }

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, bm.width(), bm.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, bm.get_data());
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glDisable(GL_TEXTURE_2D);

    set_material(value_traits::one(), value_traits::one(), value_traits::one(), value_traits::zero());
    m_type = MATERIAL_TEXTURE;

    return true;
  }

public:

  bool activate_material() const
  {
    if (MATERIAL_NONE == m_type) return false;

    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    OpenTissue::gl::ColorPicker(m_r, m_g, m_b, m_a);
    if (MATERIAL_TEXTURE == m_type) {
      glBindTexture(GL_TEXTURE_2D, m_texture);
      glEnable(GL_TEXTURE_2D);
    }
    return true;
  }

  template<typename edm_particle>
  void draw_material(edm_particle const & a0, edm_particle const & a1, edm_particle const & a2) const
  {
    if (MATERIAL_NONE == m_type) return;

    glBegin(GL_TRIANGLES);

    glNormal3f(a0.n(0), a0.n(1), a0.n(2));
    glTexCoord2f(a0.t.u, a0.t.v);
    glVertex3f(a0.r(0), a0.r(1), a0.r(2));
    
    glNormal3f(a1.n(0), a1.n(1), a1.n(2));
    glTexCoord2f(a1.t.u, a1.t.v);
    glVertex3f(a1.r(0), a1.r(1), a1.r(2));
    
    glNormal3f(a2.n(0), a2.n(1), a2.n(2));
    glTexCoord2f(a2.t.u, a2.t.v);
    glVertex3f(a2.r(0), a2.r(1), a2.r(2));

    glEnd();
  }

private:

  int  m_type;
  real_type  m_r, m_g, m_b, m_a;
  GLuint  m_texture;

};


class Application 
  : public OpenTissue::glut::PerspectiveViewApplication
{
public:

  typedef OpenTissue::math::BasicMathTypes<double, int>  math_types;
  typedef EDMMaterial<math_types>                        edm_material;
  
  typedef OpenTissue::edm::Types<math_types, edm_material, edm_material>  edm_types;

protected:

  typedef OpenTissue::edm::Surface<edm_types> surface_type;
  typedef OpenTissue::edm::Solid<edm_types>   solid_type;

  // TODO: Comment these members
  edm_types::system_type   m_sys;
  edm_types::model_type *  m_sel_model;
  size_t                   m_sel_idx;
  edm_types::force_type *  m_F;
  edm_types::real_type     m_k;
  edm_types::real_type     m_strength;

  OpenTissue::utility::Timer<double>        m_timer;
  OpenTissue::utility::FPSCounter<double>   m_fps_timer;
  
  bool m_show_fps;
  char const * m_scene;   ///< the last good known scene configuration (reset feature)
  bool m_action[256];          ///< Boolean array used to keep track of the user actions/selections.
  bool m_playing_god;
  
  static unsigned char const SPACE_KEY = ' ';
  static unsigned char const D_KEY     = 'd';
  static unsigned char const W_KEY     = 'w';

protected:


  bool reset()
  {
    m_sys.reset();
    //if (m_scene && !OpenTissue::edm::edm_system_xml_read(m_sys, m_scene))
    //{
    //  m_scene = 0;
    //  std::cerr << "\nThe provided configuration file cannot be used to create the edm system!"
    //    << "\n- The default scene will be employed." << std::endl;
    //}
    m_scene = 0;  // temporary as long as edm_system_xml_read is obsolete
    if (!m_scene && !create_default_scene())
    {
      std::cerr << "\nERROR! Internal EDM error - creation of default scene failed!" << std::endl;
      return false;
    }
    if (m_sys.model_count())
      m_sys.get_model(0).strength() = m_strength;
    return true;
  }


  bool fetch( long x, long y )
  {
    // if we've already fetched a particle, then move it!
    if ( m_sel_model )
    {
      GLdouble mm[ 16 ];
      GLdouble pm[ 16 ];
      GLint vp[ 4 ];

      glGetDoublev( GL_MODELVIEW_MATRIX, mm );
      glGetDoublev( GL_PROJECTION_MATRIX, pm );
      glGetIntegerv( GL_VIEWPORT, vp );

      edm_types::vector3_type const & p0 = m_sel_model->position(m_sel_idx);
      edm_types::vector3_type p1;
      GLdouble sx, sy, sz;
      gluProject( p0[ 0 ], p0[ 1 ], p0[ 2 ], mm, pm, vp, &sx, &sy, &sz );
      
      gluUnProject( 1.*x, 1.*(vp[ 3 ] - y + 1), sz, mm, pm, vp, &p1[ 0 ], &p1[ 1 ], &p1[ 2 ] );
      if (edm_types::EDM_Solid == m_sel_model->type()) {
        edm_types::Particle const & a = m_sel_model->particle(m_sel_idx);
        if (m_F) m_sel_model->remove(a, *m_F);
        delete m_F;
        m_F = new OpenTissue::edm::Spring<edm_types>;
        static_cast<OpenTissue::edm::Spring<edm_types>*>(m_F)->set(m_k, p1);
        m_sel_model->add(a, *m_F);
        m_sel_model->unlock_particle(m_sel_idx);
      }
      else
        m_sel_model->move_particle(m_sel_idx, p1-p0);

      return true;
    }

    // try to find a particle
    double mx[ 16 ];   // A primitive Matrix
    glMatrixMode( GL_PROJECTION );
    glPushMatrix();  // save our original projection matrix, contains stuff like front/back clipping plane, view, etc.
    glGetDoublev( GL_MODELVIEW_MATRIX, mx );
    glMultMatrixd( mx );  // Calculate the projection
    glGetDoublev( GL_PROJECTION_MATRIX, mx );
    glPopMatrix();   // restore our projection matrix or else the system will go berserk ;)

    double const e = 0.0025;  // squared radius in where a accepted selection can occur

    double vp[ 4 ];
    glGetDoublev( GL_VIEWPORT, vp );
    double ix = 2. * ( x - vp[ 0 ] - .5 * vp[ 2 ] ) / vp[ 2 ];  // X Screen to View volume
    double iy = -2. * ( y - vp[ 1 ] - .5 * vp[ 3 ] ) / vp[ 3 ];  // Y Screen to View volume

    double sel_z = 1.;  // We're now in viewport coordinates - all three axis range from [-1..1]
    size_t const model_cnt = m_sys.model_count();
    for ( size_t m = 0; m < model_cnt; ++m )
    {
      edm_types::model_type & model = m_sys.get_model( m );
      size_t const pars = model.num_particles();
      for ( size_t p = 0; p < pars; ++p )
      {
        edm_types::vector3_type const & r = model.position( p );
        double const inv_w = 1. / ( mx[ 3 ] * r[ 0 ] + mx[ 7 ] * r[ 1 ] + mx[ 11 ] * r[ 2 ] + mx[ 15 ] );
        double const x1 = inv_w * ( mx[ 0 ] * r[ 0 ] + mx[ 4 ] * r[ 1 ] + mx[ 8 ] * r[ 2 ] + mx[ 12 ] );
        double const y1 = inv_w * ( mx[ 1 ] * r[ 0 ] + mx[ 5 ] * r[ 1 ] + mx[ 9 ] * r[ 2 ] + mx[ 13 ] );
        double const z1 = inv_w * ( mx[ 2 ] * r[ 0 ] + mx[ 6 ] * r[ 1 ] + mx[ 10 ] * r[ 2 ] + mx[ 14 ] );

        if ( ( x1 - ix ) * ( x1 - ix ) + ( y1 - iy ) * ( y1 - iy ) <= e && z1 < sel_z )
        {
          m_sel_idx = p;  // just store the best selected particle!
          m_sel_model = &model;
          sel_z = z1;
        }
      }
    }

    if ( m_sel_model && (edm_types::EDM_Solid == m_sel_model->type() ? !m_action[SPACE_KEY] : true) )
      m_sel_model->lock_particle( m_sel_idx );

    return m_sel_model != 0;
  }

  void release()
  {
    if (m_sel_model) 
    {
      if (m_F) 
      {
        m_sel_model->remove(m_sel_model->particle(m_sel_idx), *m_F);
        delete m_F;
        m_F = 0;
      }
      if (edm_types::EDM_Solid == m_sel_model->type() && !m_action[SPACE_KEY])
        m_sel_model->lock_particle(m_sel_idx);
      else if (edm_types::EDM_Solid != m_sel_model->type() && m_action[SPACE_KEY])
        m_sel_model->unlock_particle(m_sel_idx);
    }
    m_sel_model = 0;
  }

  bool create_default_scene()
  {
    typedef OpenTissue::edm::Gravity<edm_types>               gravity_type;
    typedef OpenTissue::geometry::Plane<math_types>              EDMPlane;
    typedef OpenTissue::geometry::Sphere<math_types>             EDMSphere;
//    typedef OpenTissue::geometry::Torus<math_types>              EDMTorus;
//    typedef OpenTissue::geometry::Capsule<math_types>            EDMCapsule;
    typedef edm_types::object_type                                 object_type;
//    typedef OpenTissue::edm::QuadraticBezierPatch<edm_types>  quadratic_bezier_patch_type;
    typedef OpenTissue::edm::LinearBezierSolid<edm_types>     linear_bezier_solid_type;


    m_sys.create_object<EDMPlane>("Plane")->set(edm_types::vector3_type(0.0, 0.0, 1.0), edm_types::vector3_type(0.0, 0.0, -6.0));
    m_sys.create_object<EDMSphere>("Sphere")->set(edm_types::vector3_type(0.5, 0.2, -6.0), 3.0);
//    m_sys.create_object<EDMTorus>("Torus")->set(edm_types::vector3_type(0.5, 0.2, -6.0), 3.0, 1.5);
//    m_sys.create_object<EDMCapsule>("Capsule")->set(edm_types::vector3_type(2.0, 0.5, -3.5), edm_types::vector3_type(-2.0, 0.5, -3.5), 1.5);

    m_sys.get_object("Plane")->set_material(0.0, 0.0, 0.5);
    m_sys.get_object("Sphere")->set_material(0.6, 0.05, 1.0);
//    m_sys.get_object("Torus")->set_material(0.8, 0.075, 1.0);
//    m_sys.get_object("Capsule")->set_material(0.4, 0.8, 0.05);

    m_sys.create_force<gravity_type>("Gravity")->set(edm_types::vector3_type(0.0, 0.0, -9.82));

    linear_bezier_solid_type & solid = *m_sys.create_model<linear_bezier_solid_type>("Solid");
    solid.timestep() = 0.01;
    for (int i = 0; i < 8; ++i)
      solid.set_natural_position(i, edm_types::vector3_type(2*(i&0x1)-1, (i&0x2)-1, 0.5*(i&0x4)-1));
    if (!solid.initialize(3, 3, 3))
      return false;
    edm_types::tensor2_type sp; sp.t0[0] = sp.t0[1] = sp.t1[0] = sp.t1[1] = 3;
    edm_types::tensor3_type tn; tn.t0[0] = tn.t1[1] = tn.t2[2] = 3; tn.t0[1] = tn.t0[2] = tn.t1[0] = tn.t1[2] = tn.t2[0] = tn.t2[1] = 1.5;
    for (int l = 0; l < 3; ++l)
      for (int m = 0; m < 3; ++m)
        for (int n = 0; n < 3; ++n) {
          solid.set_mass(l, m, n, 1);
          solid.set_damping(l, m, n, 2);
          solid.set_tension(l, m, n, tn);
          solid.set_spatial(l, m, n, sp);
        }
    //solid.set_material("evergreen.bmp");
    solid.set_material(0.0375, 0.75, 0.45);
    solid.add(*m_sys.get_force("Gravity"));
    solid.add(*m_sys.get_object("Plane"));
    solid.add(*m_sys.get_object("Sphere"));
//    solid.add(*m_sys.get_object("Torus"));
//    solid.add(*m_sys.get_object("Capsule"));

    //quadratic_bezier_patch_type & surface = *m_sys.create_model<QuadraticBezierPatch>("Surface");
    //surface.timestep() = 0.01;
    //for (int j = 0; j < 3; ++j)
    //  for (int i = 0; i < 3; ++i)
    //    surface.set_natural_position(j*3+i, edm_types::vector3_type(i-1, 0.0, j-1));
    //if (!surface.initialize(5, 5))
    //  return false;
    //edm_types::tensor2_type tn2; tn2.t0[0] = tn2.t0[1] = tn2.t1[0] = tn2.t1[1] = 2.0;
    //edm_types::tensor2_type rg; rg.t0[0] = rg.t0[1] = rg.t1[0] = rg.t1[1] = 0.02;
    //for (int m = 0; m < 5; ++m)
    //  for (int n = 0; n < 5; ++n) {
    //    surface.set_mass(m, n, 1);
    //    surface.set_damping(m, n, 2);
    //    surface.set_tension(m, n, tn2);
    //    surface.set_rigidity(m, n, rg);
    //  }
    ////surface.set_material("evergreen.bmp");
    //surface.set_material(0.0375, 0.75, 0.45);
    //surface.add(*m_sys.get_force("Gravity"));
    //surface.add(*m_sys.get_object("Plane"));
    //surface.add(*m_sys.get_object("Sphere"));
    //surface.add(*m_sys.get_object("Torus"));
    //surface.add(*m_sys.get_object("Capsule"));

    return true;
  }

public:


  Application(int argc, char **argv)
    : m_sel_model(0)
    , m_sel_idx(0)
    , m_F(0)
    , m_k(boost::numeric_cast<edm_types::real_type>(1250))
    , m_strength(boost::numeric_cast<edm_types::real_type>(1))
    , m_show_fps(false)
    , m_scene(0)
  {
    this->width() = 800;
    this->height() = 450;
    memset(m_action, false, sizeof(bool)*256);
    if (argc > 1)
      m_scene = argv[1];
  }

public:

  char const * do_get_title() const { return "Elastically Deformable Models Demo Application"; }

  void do_display()
  {
    // draw all models
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glDisable(GL_LIGHT1);
    glDisable(GL_TEXTURE_2D);

    // draw all objects
    edm_types::system_type::EDMIOObjects const & objects = m_sys.objects();
    edm_types::system_type::EDMIOObjects::const_iterator object;
    for (object = objects.begin(); object != objects.end(); ++object) {
      edm_types::object_type const & obj = *object->second;
      if (!obj.get_visibility())
        continue;
      if (!obj.activate_material())
        glDisable(GL_LIGHTING);
      bool const wireframe = edm_material::MATERIAL_NONE == obj.get_material_type();

      typedef OpenTissue::geometry::Sphere<math_types> EDMSphere;
      typedef OpenTissue::geometry::Capsule<math_types> EDMCapsule;
      typedef OpenTissue::geometry::Torus<math_types> EDMTorus;
      typedef OpenTissue::geometry::Plane<math_types> EDMPlane;

      if (EDMSphere::id() == obj.get_shape()->class_id())
        OpenTissue::gl::DrawSphere(*static_cast<EDMSphere const*>(obj.get_shape()), wireframe);
      else if (EDMCapsule::id() == obj.get_shape()->class_id())
        OpenTissue::gl::DrawCapsule(*static_cast<EDMCapsule const*>(obj.get_shape()), wireframe);
      else if (EDMTorus::id() == obj.get_shape()->class_id())
        OpenTissue::gl::DrawTorus(*static_cast<EDMTorus const*>(obj.get_shape()), wireframe);
      else if (EDMPlane::id() == obj.get_shape()->class_id())
        OpenTissue::gl::DrawPlane(*static_cast<EDMPlane const*>(obj.get_shape()), wireframe);
    }

    edm_types::system_type::EDMIOModels const & models = m_sys.models();
    edm_types::system_type::EDMIOModels::const_iterator model;
    for (model = models.begin(); model != models.end(); ++model) {
      edm_types::model_type const * m = model->second;
      switch (m->type()) 
      {
        case edm_types::EDM_Surface : draw_surface(*static_cast<surface_type const*>(m)); break;
        case edm_types::EDM_Solid : draw_solid(*static_cast<solid_type const*>(m)); break;
      }
      glDisable( GL_TEXTURE_2D );
      glEnable( GL_COLOR_MATERIAL );
      // draw visual debug
      if ( m_action[ D_KEY ] )
      {
        GLUquadric *qobj = gluNewQuadric();
        gluQuadricDrawStyle( qobj, GLU_FILL );
        for ( size_t n = 0; n < m->num_particles(); ++n )
        {
          edm_types::Particle const & a = m->particle( n );
          // draw particles
          glPushMatrix();
          glTranslated( a.r[ 0 ], a.r[ 1 ], a.r[ 2 ] );
          if ( a.f )   // draw fixed particles in dark red
            OpenTissue::gl::ColorPicker( 0.7, 0.0, 0.0 );
          else  // the other particles in dark grey
            OpenTissue::gl::ColorPicker(0.4, 0.4, 0.4 );
          glEnable( GL_LIGHTING );
          gluSphere( qobj, 0.05, 12, 12 );
          glPopMatrix();

          // draw normals
          glDisable( GL_LIGHTING );
          OpenTissue::gl::ColorPicker(0.0, 0.0, 0.0);
          glBegin( GL_LINES );
          edm_types::vector3_type const nor = a.r + .25 * a.n;
          glVertex3d( a.r[ 0 ], a.r[ 1 ], a.r[ 2 ] );
          glVertex3d( nor[ 0 ], nor[ 1 ], nor[ 2 ] );
          glEnd();

          // draw external force as GREEN
          OpenTissue::gl::ColorPicker(0.0, 1.0, 0.0);
          glBegin( GL_LINES );
          edm_types::vector3_type const f = a.r + .25 * a.F;
          glVertex3d( a.r[ 0 ], a.r[ 1 ], a.r[ 2 ] );
          glVertex3d( f[ 0 ], f[ 1 ], f[ 2 ] );
          glEnd();

          // draw elasticity force as PURPLE
          OpenTissue::gl::ColorPicker(1.0, 0.0, 1.0);
          glBegin( GL_LINES );
          edm_types::vector3_type const e = a.r + .25 * a.E;
          glVertex3d( a.r[ 0 ], a.r[ 1 ], a.r[ 2 ] );
          glVertex3d( e[ 0 ], e[ 1 ], e[ 2 ] );
          glEnd();
        }
        gluDeleteQuadric( qobj );
      }
    }

  }

  void do_action(unsigned char choice)
  {
    using std::min;
    using std::max;

    // Toggle state
    m_action[ choice ] = !m_action[ choice ];
    switch ( choice )
    {
    case 'd':
      std::cout << "visual debug: " << ( m_action[ choice ] ? "ON" : "OFF" ) << std::endl;
      //m_sys.visual_debug( m_action[ choice ] );
      break;
    case 'w':
      std::cout << "wireframe: " << ( m_action[ choice ] ? "ON" : "OFF" ) << std::endl;
      //m_sys.show_wireframe( m_action[ choice ] );
      break;
    case 'r':
      std::cout << "resetting system" << std::endl;
      reset();
      m_action[ choice ] = false;
      break;
    case 'f':
      m_show_fps = m_action[ choice ];
      std::cout << "frames per second (FPS): " << ( m_show_fps ? "ON" : "OFF" ) << std::endl;
      break;
    case '+':
      m_strength = min( boost::numeric_cast<edm_types::real_type>(4), m_strength + boost::numeric_cast<edm_types::real_type>(0.05) );
      m_sys.get_model( 0 ).strength() = m_strength;
      std::cout << "strength: " << m_strength << std::endl;
      break;
    case '-':
      m_strength = max( boost::numeric_cast<edm_types::real_type>(0), m_strength - boost::numeric_cast<edm_types::real_type>(0.05) );
      m_sys.get_model( 0 ).strength() =  m_strength;
      std::cout << "strength: " << m_strength << std::endl;
      break;
    case ' ':
      m_timer.start();
      break;
    } // End Switch
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry)){}

  void do_init()
  {
    math_types::vector3_type position(0,-20,2);
    math_types::vector3_type target(0,2,-4);
    math_types::vector3_type up(0,0,1);
    this->camera().init(position, target, up);
 
    m_playing_god = false;
    reset();
  }

  void do_run()
  {
    static double cnt = 0.;
    double const sync = 1./60.;;  // 30Hz
    m_timer.stop();
    cnt += m_timer();
    m_timer.start();
    if (cnt >= sync) {  // run as fast as possible but not more than sync
      m_sys.run(m_action[ D_KEY ]);
      cnt -= sync;
      if (m_fps_timer.frame())
        if ( m_show_fps )
          std::cout << "Simulation FPS: " << m_fps_timer() << std::endl;
    }
  }

  void do_shutdown(){}

  void mouse_down(double cur_x,double cur_y,bool shift,bool ctrl,bool alt,bool left,bool middle,bool right) 
  {
    OpenTissue::glut::PerspectiveViewApplication::mouse_down(cur_x, cur_y, shift, ctrl, alt, left, middle, right);
    if ( ctrl && left )
    {
      this->fetch( boost::numeric_cast<long>(cur_x), boost::numeric_cast<long>(cur_y) );
      glutPostRedisplay();
      m_playing_god = true;
    }
  }

  void mouse_up(double cur_x,double cur_y,bool shift,bool ctrl,bool alt,bool left,bool middle,bool right) 
  {
    OpenTissue::glut::PerspectiveViewApplication::mouse_up(cur_x, cur_y, shift, ctrl, alt, left, middle, right);
    if ( m_playing_god && left )
    {
      this->release();
      m_playing_god = false;
    }
  }

  void mouse_move(double cur_x,double cur_y) 
  {
    OpenTissue::glut::PerspectiveViewApplication::mouse_move(cur_x, cur_y);
    if ( m_playing_god )
      this->fetch( boost::numeric_cast<long>(cur_x), boost::numeric_cast<long>(cur_y) );
  }

  void draw_surface(surface_type const & surface) const
  {
    edm_types::model_type const & model = *static_cast<edm_types::model_type const*>(&surface);

    glDisable( GL_LIGHT1 );
    glEnable( GL_LIGHT0 );
    if (model.activate_material()) {
      for (size_t n = 1; n < surface.get_num_N(true); ++n )
        for ( size_t m = 1; m < surface.get_num_M(true); ++m )
        {
          edm_types::Particle const & a0 = model.particle(surface.index_adjust( m - 1, n - 1 ));
          edm_types::Particle const & a1 = model.particle(surface.index_adjust( m - 1, n ));
          edm_types::Particle const & a2 = model.particle(surface.index_adjust( m, n ));
          edm_types::Particle const & a3 = model.particle(surface.index_adjust( m, n - 1 ));
          model.draw_material( a0, a2, a1 );
          model.draw_material( a0, a3, a2 );
        }
    }

    glDisable( GL_LIGHTING );
    glDisable( GL_TEXTURE_2D );
    if ( m_action[ W_KEY ] || edm_material::MATERIAL_NONE == model.get_material_type() )   // draw surface wireframe triangles in BLUE
    {
      OpenTissue::gl::ColorPicker( 0.0, 0.0, 0.8 );
      for ( size_t n = 1; n < surface.get_num_N(true); ++n )
        for ( size_t m = 1; m < surface.get_num_M(true); ++m )
        {
          edm_types::vector3_type const & p0 = model.position(surface.index_adjust( m - 1, n - 1 ));
          edm_types::vector3_type const & p1 = model.position(surface.index_adjust( m - 1, n ));
          edm_types::vector3_type const & p2 = model.position(surface.index_adjust( m, n ));
          edm_types::vector3_type const & p3 = model.position(surface.index_adjust( m, n - 1 ));
          glBegin( GL_LINE_STRIP );
          glVertex3d( p0[ 0 ], p0[ 1 ], p0[ 2 ] );
          glVertex3d( p1[ 0 ], p1[ 1 ], p1[ 2 ] );
          glVertex3d( p2[ 0 ], p2[ 1 ], p2[ 2 ] );
          glVertex3d( p0[ 0 ], p0[ 1 ], p0[ 2 ] );
          glVertex3d( p3[ 0 ], p3[ 1 ], p3[ 2 ] );
          glVertex3d( p2[ 0 ], p2[ 1 ], p2[ 2 ] );
          glEnd();
        }
    }
  }

  void draw_solid(solid_type const & solid) const
  {
    edm_types::model_type const & model = *static_cast<edm_types::model_type const*>(&solid);

    glDisable( GL_LIGHT1 );
    glEnable( GL_LIGHT0 );
    if (model.activate_material()) {
      // draw face l == 0
      for ( size_t n = 1; n < solid.get_num_N(true); ++n )
        for ( size_t m = 1; m < solid.get_num_M(true); ++m )
        {
          edm_types::Particle const & a0 = model.particle(solid.index_adjust( 0, m - 1, n - 1 ));
          edm_types::Particle const & a1 = model.particle(solid.index_adjust( 0, m - 1, n ));
          edm_types::Particle const & a2 = model.particle(solid.index_adjust( 0, m, n ));
          edm_types::Particle const & a3 = model.particle(solid.index_adjust( 0, m, n - 1 ));
          model.draw_material( a0, a2, a1 );
          model.draw_material( a0, a3, a2 );
        }
      // draw face l == L-1
      for ( size_t n = 1; n < solid.get_num_N(true); ++n )
        for ( size_t m = 1; m < solid.get_num_M(true); ++m )
        {
          edm_types::Particle const & a0 = model.particle(solid.index_adjust( solid.get_num_L() - 1, m - 1, n - 1 ));
          edm_types::Particle const & a2 = model.particle(solid.index_adjust( solid.get_num_L() - 1, m, n ));
          edm_types::Particle const & a1 = model.particle(solid.index_adjust( solid.get_num_L() - 1, m - 1, n ));
          edm_types::Particle const & a3 = model.particle(solid.index_adjust( solid.get_num_L() - 1, m, n - 1 ));
          model.draw_material( a0, a1, a2 );
          model.draw_material( a0, a2, a3 );
        }
      // draw face m == 0
      for ( size_t n = 1; n < solid.get_num_N(true); ++n )
        for ( size_t l = 1; l < solid.get_num_L(true); ++l )
        {
          edm_types::Particle const & a0 = model.particle(solid.index_adjust( l - 1, 0, n - 1 ));
          edm_types::Particle const & a1 = model.particle(solid.index_adjust( l - 1, 0, n ));
          edm_types::Particle const & a2 = model.particle(solid.index_adjust( l, 0, n ));
          edm_types::Particle const & a3 = model.particle(solid.index_adjust( l, 0, n - 1 ));
          model.draw_material( a0, a1, a2 );
          model.draw_material( a0, a2, a3 );
        }
      // draw face m == M-1
      for ( size_t n = 1; n < solid.get_num_N(true); ++n )
        for ( size_t l = 1; l < solid.get_num_L(true); ++l )
        {
          edm_types::Particle const & a0 = model.particle(solid.index_adjust( l - 1, solid.get_num_M() - 1, n - 1 ));
          edm_types::Particle const & a2 = model.particle(solid.index_adjust( l, solid.get_num_M() - 1, n ));
          edm_types::Particle const & a1 = model.particle(solid.index_adjust( l - 1, solid.get_num_M() - 1, n ));
          edm_types::Particle const & a3 = model.particle(solid.index_adjust( l, solid.get_num_M() - 1, n - 1 ));
          model.draw_material( a0, a2, a1 );
          model.draw_material( a0, a3, a2 );
        }
      // draw face n == 0
      for ( size_t m = 1; m < solid.get_num_M(true); ++m )
        for ( size_t l = 1; l < solid.get_num_L(true); ++l )
        {
          edm_types::Particle const & a0 = model.particle(solid.index_adjust( l - 1, m - 1, 0 ));
          edm_types::Particle const & a1 = model.particle(solid.index_adjust( l - 1, m, 0 ));
          edm_types::Particle const & a2 = model.particle(solid.index_adjust( l, m, 0 ));
          edm_types::Particle const & a3 = model.particle(solid.index_adjust( l, m - 1, 0 ));
          model.draw_material( a0, a2, a1 );
          model.draw_material( a0, a3, a2 );
        }
      // draw face n == N-1
      for ( size_t m = 1; m < solid.get_num_M(true); ++m )
        for ( size_t l = 1; l < solid.get_num_L(true); ++l )
        {
          edm_types::Particle const & a0 = model.particle(solid.index_adjust( l - 1, m - 1, solid.get_num_N() - 1 ));
          edm_types::Particle const & a2 = model.particle(solid.index_adjust( l, m, solid.get_num_N() - 1 ));
          edm_types::Particle const & a1 = model.particle(solid.index_adjust( l - 1, m, solid.get_num_N() - 1 ));
          edm_types::Particle const & a3 = model.particle(solid.index_adjust( l, m - 1, solid.get_num_N() - 1 ));
          model.draw_material( a0, a1, a2 );
          model.draw_material( a0, a2, a3 );
        }
    }

    glDisable( GL_LIGHTING );
    glDisable( GL_TEXTURE_2D );
    if ( m_action[ W_KEY ] || edm_material::MATERIAL_NONE == model.get_material_type() )   // draw surface wireframe triangles in BLUE
    {
      OpenTissue::gl::ColorPicker( 0.0, 0.0, 0.8 );
      for ( size_t n = 0; n < solid.get_num_N(true); ++n )
        for ( size_t m = 0; m < solid.get_num_M(true); ++m )
          for ( size_t l = 0; l < solid.get_num_L(true); ++l )
          {
            if ( ( l == 0 || l == solid.get_num_L() - 1 ) && n > 0 && m > 0 )
            {
              edm_types::vector3_type const & p0 = model.position(solid.index_adjust( l, m - 1, n - 1 ));
              edm_types::vector3_type const & p1 = model.position(solid.index_adjust( l, m - 1, n ));
              edm_types::vector3_type const & p2 = model.position(solid.index_adjust( l, m, n ));
              edm_types::vector3_type const & p3 = model.position(solid.index_adjust( l, m, n - 1 ));
              glBegin( GL_LINE_LOOP );
              glVertex3d( p0[ 0 ], p0[ 1 ], p0[ 2 ] );
              glVertex3d( p1[ 0 ], p1[ 1 ], p1[ 2 ] );
              glVertex3d( p2[ 0 ], p2[ 1 ], p2[ 2 ] );
              glVertex3d( p3[ 0 ], p3[ 1 ], p3[ 2 ] );
              glEnd();
            }
            if ( ( m == 0 || m == solid.get_num_M() - 1 ) && n > 0 && l > 0 )
            {
              edm_types::vector3_type const & p0 = model.position(solid.index_adjust( l - 1, m, n - 1 ));
              edm_types::vector3_type const & p1 = model.position(solid.index_adjust( l - 1, m, n ));
              edm_types::vector3_type const & p2 = model.position(solid.index_adjust( l, m, n ));
              edm_types::vector3_type const & p3 = model.position(solid.index_adjust( l, m, n - 1 ));
              glBegin( GL_LINE_LOOP );
              glVertex3d( p0[ 0 ], p0[ 1 ], p0[ 2 ] );
              glVertex3d( p1[ 0 ], p1[ 1 ], p1[ 2 ] );
              glVertex3d( p2[ 0 ], p2[ 1 ], p2[ 2 ] );
              glVertex3d( p3[ 0 ], p3[ 1 ], p3[ 2 ] );
              glEnd();
            }
            if ( ( n == 0 || n == solid.get_num_N() - 1 ) && l > 0 && m > 0 )
            {
              edm_types::vector3_type const & p0 = model.position(solid.index_adjust( l - 1, m - 1, n ));
              edm_types::vector3_type const & p1 = model.position(solid.index_adjust( l - 1, m, n ));
              edm_types::vector3_type const & p2 = model.position(solid.index_adjust( l, m, n ));
              edm_types::vector3_type const & p3 = model.position(solid.index_adjust( l, m - 1, n ));
              glBegin( GL_LINE_LOOP );
              glVertex3d( p0[ 0 ], p0[ 1 ], p0[ 2 ] );
              glVertex3d( p1[ 0 ], p1[ 1 ], p1[ 2 ] );
              glVertex3d( p2[ 0 ], p2[ 1 ], p2[ 2 ] );
              glVertex3d( p3[ 0 ], p3[ 1 ], p3[ 2 ] );
              glEnd();
            }
          }
    }
  }

};

OpenTissue::glut::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::glut::instance_pointer instance;
  instance.reset( new Application(argc,argv) );

  return instance;
}
