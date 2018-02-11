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
#include <OpenTissue/core/math/math_precision.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_compute_smallest_sphere.h>
#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/core/geometry/geometry_aabb_fit.h>
#include <OpenTissue/core/geometry/geometry_obb.h>
#include <OpenTissue/core/geometry/geometry_obb_fit.h>
#include <OpenTissue/core/geometry/geometry_cylinder.h>
#include <OpenTissue/core/geometry/geometry_cylinder_fit.h>
#include <OpenTissue/core/geometry/geometry_prism.h>
#include <OpenTissue/core/geometry/geometry_prism_fit.h>
#include <OpenTissue/core/geometry/geometry_hybrid_fit.h>
#include <cmath>


class Application : public OpenTissue::glut::PerspectiveViewApplication
{
protected:

  typedef OpenTissue::math::BasicMathTypes<double,size_t>   math_types;

  typedef math_types::value_traits   value_traits;
  typedef math_types::real_type      real_type;
  typedef math_types::vector3_type   vector3_type;
  typedef math_types::matrix3x3_type matrix3x3_type;

  OpenTissue::geometry::HybridVolume<math_types> m_hybrid;
  OpenTissue::geometry::AABB<math_types>         m_aabb;
  OpenTissue::geometry::Sphere<math_types>       m_sphere;
  OpenTissue::geometry::OBB<math_types>          m_obb;
  OpenTissue::geometry::Cylinder<math_types>     m_cylinder;
  OpenTissue::geometry::Prism<math_types>        m_prism;
  vector3_type                         m_points[1000];
  unsigned int                         m_count;

public:

  Application()
    : m_count(100)
  { }

public:

  char const * do_get_title() const { return "Volume Fitting Demo Application"; }

  void do_display()
  {
    bool wireframe = true;
    OpenTissue::gl::ColorPicker(1,0,0);
    OpenTissue::gl::DrawAABB(m_aabb, wireframe);
    OpenTissue::gl::ColorPicker(0,1,0);
    OpenTissue::gl::DrawSphere(m_sphere, wireframe);
    OpenTissue::gl::ColorPicker(0,0,1);
    OpenTissue::gl::DrawOBB(m_obb, wireframe);
    OpenTissue::gl::ColorPicker(1,0,1);
    OpenTissue::gl::DrawCylinder(m_cylinder, wireframe);
    OpenTissue::gl::ColorPicker(0,1,1);
    OpenTissue::gl::DrawPrism(m_prism, wireframe);
    OpenTissue::gl::ColorPicker(1,0,0);
    OpenTissue::gl::DrawHybrid(m_hybrid, wireframe);
    GLUquadricObj* qobj = gluNewQuadric();
    OpenTissue::gl::ColorPicker(1,1,0);
    for(unsigned int i=0;i<m_count;++i)
    {
      glPushMatrix();
      glTranslatef(
        (float)m_points[i](0),
        (float)m_points[i](1),
        (float)m_points[i](2)
        );
      gluSphere(qobj,0.1,12,12);
      glPopMatrix();
    }
    gluDeleteQuadric(qobj);
  }

  void do_action(unsigned char choice)
  {
    using std::fabs;

    switch(choice)
    {
    case 'r':
      {
        matrix3x3_type R;
        vector3_type disp;
        random(disp,-value_traits::pi(), value_traits::pi());
        vector3_type n;
        OpenTissue::math::random(n);
        n = OpenTissue::math::unit(n);
        R = OpenTissue::math::Ru(disp(0),n);

        static real_type eps = OpenTissue::math::working_precision<real_type>();
        for(unsigned int i=0;i<m_count;++i)
        {
          vector3_type tmp;
          real_type lower = -fabs(disp(0))-eps;
          real_type upper = fabs(disp(0))+eps;
          OpenTissue::math::random(tmp,lower,upper);
          m_points[i] = R * tmp;
          m_points[i] += disp;
        }
      }
      break;
    case 'a':  OpenTissue::geometry::aabb_fit(m_points, m_points+m_count, m_aabb); break;
    case 'c':  OpenTissue::geometry::cylinder_fit(m_points,m_points+m_count,m_cylinder,false); break;
    case 's':  OpenTissue::geometry::compute_smallest_sphere(m_points, m_points + m_count, m_sphere); break;
    case 'b':  OpenTissue::geometry::obb_fit(m_points,m_points+m_count,m_obb,false); break;
    case 'p':  OpenTissue::geometry::prism_fit(m_points,m_points+m_count,m_prism,false); break;
    case 'h':
      {
        std::vector< OpenTissue::geometry::VolumeShape<math_types> * > volumes;
        volumes.push_back(&m_aabb);
        volumes.push_back(&m_cylinder);
        volumes.push_back(&m_obb);
        volumes.push_back(&m_prism);
        volumes.push_back(&m_sphere);
        OpenTissue::geometry::hybrid_volume_fit(volumes.begin(), volumes.end(), m_hybrid);
      }
      break;
    default:
      std::cout << "You have pressed" << choice << std::endl;
      break;
    };/*End switch*/
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int general = glutCreateMenu(menu);
    glutAddMenuEntry("Generate Random Points [r]", 'r');
    glutAddMenuEntry("Fit AABB               [a]", 'a');
    glutAddMenuEntry("Fit Cylinder           [c]", 'c');
    glutAddMenuEntry("Fit OBB                [b]", 'b');
    glutAddMenuEntry("Fit Sphere             [s]", 's');
    glutAddMenuEntry("Fit Hybrid             [h]", 'h');
    glutSetMenu(main_menu);
    glutAddSubMenu("Fitting", general);
  }

  void do_init()
  {
    this->camera().move(90);
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
