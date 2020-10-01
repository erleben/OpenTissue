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


#define SPHSH
//#define SPHSH_PARALLEL

#include <OpenTissue/dynamics/sph/sph.h>
#include <OpenTissue/core/math/math_constants.h>
#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/containers/grid/grid.h>
#include <OpenTissue/core/containers/grid/util/grid_idx2coord.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/geometry/geometry_capsule.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_obb.h>
#include <OpenTissue/collision/spatial_hashing/spatial_hashing.h>
#include <OpenTissue/utility/utility_fps_counter.h>
#include <OpenTissue/utility/utility_runtime_type.h>

#include <vector>
#include <list>
#include <cmath>
#include <string>

#include <boost/cast.hpp> // we need boost::numeric_cast<>()


using namespace OpenTissue::math;
using namespace OpenTissue::math::detail;


/**
*
* 2007-05-21 kenny: 
*
*    The compiler do not like it if these typedef's are moved inside the class
*    definition. As far as I can tell it boils down to using a pointer as a
*    template argument.  This should be re-factored!!!
*/

  template<typename Point_Type>
  class T4NodeTraits
  {
  public:
    typedef Point_Type  point_type;
    point_type const & vertex() const {return *m_vertex;}
    point_type const *  m_vertex;
  };

  typedef OpenTissue::utility::RuntimeType<double>  RTreal;
  RTreal Radius;
  typedef BasicMathTypes<double,int> math_types;

  typedef math_types::vector3_type           point;
  typedef math_types::vector3_type           vector3_type;
  typedef math_types::real_type              real_type;
  typedef std::vector<point>                 point_container;
  typedef OpenTissue::sph::Particle<real_type, Vector3, &Radius>  Particle;

  typedef OpenTissue::geometry::VolumeShape<math_types> Volume;
  typedef OpenTissue::geometry::Capsule<math_types>     CapsuleObj;
  typedef OpenTissue::geometry::Sphere<math_types>      SphereObj;
  typedef OpenTissue::geometry::OBB<math_types>         BoxObj;

  typedef OpenTissue::sph::ImplicitSpherePrimitive<real_type, vector3_type, SphereObj> ImplicitSphere;
  typedef OpenTissue::sph::ImplicitCapsulePrimitive<real_type, vector3_type, CapsuleObj> ImplicitCapsule;
  typedef OpenTissue::sph::ImplicitPlanePrimitive<real_type, vector3_type> ImplicitPlane;
  typedef OpenTissue::sph::ImplicitBoxPrimitive<real_type, vector3_type, BoxObj> ImplicitBox;

  //typedef OpenTissue::CollisionTypeBinder<real_type, OpenTissue::vector3> CollisionTypes;
  //typedef OpenTissue::ParticleWrapper<point, Particle> Particle_Wrapper;
  //typedef OpenTissue::TetrahedronWrapper<real_type, point, tetrahedra_mesh::tetrahedron_type> Tetrahedron_Wrapper;
  //typedef OpenTissue::TetrahedraPointsCollisionDetectionPolicy<CollisionTypes, Tetrahedron_Wrapper, Particle_Wrapper> CollisionDetection;
  //typedef OpenTissue::TetrahedraPointsCollisionDetectionPolicy<real_type, vector3_type, tetrahedra_mesh::tetrahedron_type, Particle> T4CollisionDetection;
  typedef OpenTissue::sph::ImplicitPrimitivesCollisionDetectionPolicy<real_type, vector3_type, Particle> IPCollisionDetection;

  typedef OpenTissue::sph::Types<
      real_type
    , Vector3
    , Particle
    , IPCollisionDetection
    , OpenTissue::spatial_hashing::PrimeNumberHashFunction
    , OpenTissue::spatial_hashing::Grid
    , OpenTissue::spatial_hashing::PointDataQuery
  > SPHTypes;


  typedef OpenTissue::sph::WFixedGaussian<SPHTypes, &Radius, false> KernelGaussian;
  typedef OpenTissue::sph::WPoly6<SPHTypes, &Radius, false> KernelDefault;
  typedef OpenTissue::sph::WSpiky<SPHTypes, &Radius, false> KernelPressure;
  typedef OpenTissue::sph::WViscosity<SPHTypes, &Radius, false> KernelViscosity;

  typedef OpenTissue::sph::Density<SPHTypes, KernelDefault> DensitySolver;
  typedef OpenTissue::sph::SurfaceNormal<SPHTypes, KernelDefault> NormalSolver;
  typedef OpenTissue::sph::PressureForce<SPHTypes, KernelPressure> PressureForce;
  typedef OpenTissue::sph::ViscosityForce<SPHTypes, KernelViscosity> ViscosityForce;
  typedef OpenTissue::sph::SurfaceForce<SPHTypes, KernelDefault> SurfaceForce;
  typedef OpenTissue::sph::ColorField<SPHTypes, KernelDefault> ColorField;

  typedef OpenTissue::sph::Pressure<SPHTypes> PressureSolver;
  typedef OpenTissue::sph::Gravity<SPHTypes> GravityForce;
  typedef OpenTissue::sph::Buoyancy<SPHTypes> BuoyancyForce;

  typedef OpenTissue::sph::Verlet<SPHTypes> VerletIntegrator;
  typedef OpenTissue::sph::Euler<SPHTypes> EulerIntegrator;
  typedef OpenTissue::sph::LeapFrog<SPHTypes> LeapFrogIntegrator;

  typedef OpenTissue::sph::Water<SPHTypes> WaterMaterial;
  typedef OpenTissue::sph::Mucus<SPHTypes> MucusMaterial;
  typedef OpenTissue::sph::Steam<SPHTypes> SteamMaterial;

  typedef OpenTissue::sph::PointEmitter<SPHTypes> PointEmitter;
  typedef OpenTissue::sph::CircleEmitter<SPHTypes> CircleEmitter;


  typedef OpenTissue::sph::System
  <   SPHTypes
    , DensitySolver
    , PressureSolver
    , NormalSolver
    , GravityForce
    , BuoyancyForce
    , PressureForce
    , ViscosityForce
    , SurfaceForce
    , LeapFrogIntegrator
    , ColorField
  > DefaultSystem;




/**
*
* 2007-05-21: kenny: outstanding issues: 
*                const-correctness, 
*                OT-naming conventions is not used (ie. m_ on members, _type on types,
*                prober placement of braces etc..),
*                messy usage of math types and ad-hoc typedefs.
*                Contains redundant code like Screen2World and putText.
*                documentation is missing too.
*                avoid if-directives in code, in particular ``if 0'' stuff
*/
class Application : public OpenTissue::graphics::PerspectiveViewApplication
{
protected:

  bool draw_normals;
  bool draw_surface;
  bool draw_inside;
  bool draw_velocity;
  bool osd;
  bool render;
  int obstacles;
  bool use_emitter;
  bool waves;

  int yScale;
  bool bPanObstacle;
  bool bScaleObstacle;
  double xPan, yPan, zPan;

protected:



  DefaultSystem* sph;
  OpenTissue::sph::Material<SPHTypes>* material;
  OpenTissue::grid::Grid<double,math_types>  phi;
  OpenTissue::polymesh::PolyMesh<> surface;

  OpenTissue::utility::FPSCounter<double> fps;
  point_container points;
  SphereObj sphere;
  SphereObj sphere2;
  BoxObj box;
  ImplicitSphere isphere;
  ImplicitSphere isphere2;
  CapsuleObj capsule;
  ImplicitCapsule icapsule;
  ImplicitPlane iplane;
  ImplicitBox ibox;
  PointEmitter emitter1;
  CircleEmitter emitter2;
  CircleEmitter emitter3;

  BoxObj DBb1;
  BoxObj DBb2;
  BoxObj DBb3;
  ImplicitBox iDBb1;
  ImplicitBox iDBb2;

  OpenTissue::sph::Emitter<SPHTypes>* emitter;
  Volume* object;

public:

  Application()
    : draw_normals(false)
    , draw_surface(false)
    , draw_inside(false)
    , draw_velocity(false)
    , osd(true)
    , render(false)
    , obstacles(0)
    , use_emitter(false)
    , waves(false)
    , bPanObstacle(false)
    , bScaleObstacle(false)
    , sph(NULL)
    , material(NULL)
    , sphere(vector3_type(0,0,0.1), 0.25)
    , sphere2(vector3_type(0,0,-0.2), 0.25)
    , box(vector3_type(0.0,0.0,0.0), diag(1.), vector3_type(0.1875,0.1875,0.1875))
    , isphere(sphere)
    , isphere2(sphere2)
    , capsule(vector3_type(0.0,0.0,0.0), vector3_type(0.0,0.0,0.6), 0.2)
    , icapsule(capsule)
    , iplane(vector3_type(0,0,-2.0), vector3_type(0,0,1.0))
    , ibox(box)
    , emitter1(vector3_type(0,0,0), vector3_type(-1.5,0,.5))
    , emitter2( vector3_type(0.0), 0.015, vector3_type(0.0,0.0,1.5))
    , emitter3(vector3_type(0,0,0.7), 0.02, vector3_type(0,0,-1))
    , DBb1(vector3_type(0.0,0.0,0.0), diag(1.), vector3_type(0.1875,0.1,0.4))
    , DBb2(vector3_type(0.3125,0.0,0.0), diag(1.), vector3_type(0.5,0.1,0.4))
    , DBb3(vector3_type(0.0,0.0,0.0), diag(1.), vector3_type(0.5,0.1,0.4))
    , iDBb1(DBb1)
    , iDBb2(DBb2)
    , emitter(&emitter2)
    , object(&capsule)
  { }

protected:

  /**
  * Put text onto screen (OSD)
  *
  * 2007-05-21 kenny: what is wrong with OpenTissue::gl::DrawString()?
  */
  void putText(const double &x, const double &y, const double &r, const double &g, const double &b, const std::string &text)
  {
    // setup orthogonal projection to use 2D text
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glViewport(0, 0, this->width(), this->height());
    glOrtho(0, this->width(), 0, this->height(), 0, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glDisable(GL_TEXTURE_2D);

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


  void renderParticles(const DefaultSystem::fluid_material* mat)
  {
    GLUquadric * qobj = gluNewQuadric();
    gluQuadricDrawStyle(qobj, GLU_FILL);
    glEnable(GL_COLOR_MATERIAL);
    glDisable(GL_LIGHTING);

    //    unsigned long cnt = 0;
    const SPHTypes::particle_container &pars = sph->particles();
    for (SPHTypes::particle_container::const_iterator p = pars.begin(); p != pars.end(); p++) {
      if (p->fixed()) continue;
      const vector3_type &pos = p->position();

      //
      // V = 4/3 pi r^3  =>  m/rho = 4/3 pi r^3  =>  r^3 = (3 m)/(4 pi rho)  =>  r = ((3 m)/(4 pi rho))^1/3
      //
      const SPHTypes::real_type d = 0.9*pow((3.*p->mass())/(4.*pi<real_type>()*p->density()), 1./3.);
      const vector3_type& n = p->normal();
      const SPHTypes::real_type l = n*n;

      bool draw_sphere = false;
      if (l >= mat->threshold() || l < 0.0025) 
      {
        // draw surface normals
        if (draw_normals && l >= mat->threshold()) 
        {
          const vector3_type sn = -0.05*unit(n);
          glColor3d(0.5, 0.5, 0.5);
          glBegin(GL_LINES);
          glVertex3d(pos[0], pos[1], pos[2]);
          glVertex3d(pos[0]+sn[0], pos[1]+sn[1], pos[2]+sn[2]);
          glEnd();
        }
        glColor3d(mat->red(), mat->green(), mat->blue());
        draw_sphere = draw_surface;
      }
      else 
      {
        glColor3d(.5*mat->red(), .5*mat->green(), .5*mat->blue());
        draw_sphere = draw_inside;
      }

      // default behaviour
      if (!(draw_surface || draw_inside)) 
      {
        glColor3d(mat->red(), mat->green(), mat->blue());
        draw_sphere = true;
      }

      if (draw_velocity) 
      {
        glEnable(GL_LIGHTING);
        const vector3_type vel = 0.05*p->velocity();
        OpenTissue::gl::DrawVector(pos, vel, 1, false);
      }
      else if (draw_sphere) 
      {
        glPushMatrix();
        glTranslated(pos[0], pos[1], pos[2]);
        glEnable(GL_LIGHTING);
        gluSphere(qobj, d, 8, 8);
        glDisable(GL_LIGHTING);
        glPopMatrix();
      }

    }
    gluDeleteQuadric(qobj);
  }


  void renderSurface(const DefaultSystem::fluid_material* mat, const SPHTypes::real_type& size)
  {
    using std::max;
    using std::min;

    typedef SPHTypes::real_type   real_type;
    typedef vector3_type vector3_type;
    // find BB of fluid (for high res rendering)
    vector3_type  m( highest<real_type>() ), M( lowest<real_type>() );

    const SPHTypes::particle_container &pars = sph->particles();
    SPHTypes::particle_container::const_iterator pars_end = pars.end();
    for (SPHTypes::particle_container::const_iterator p = pars.begin(); p != pars_end; ++p)
    {
      vector3_type const & pos = p->position();
      M = max(M, pos);
      m = min(m, pos);
    }
    real_type const d = 1.35*pow((3.*mat->particle_mass())/(4.*pi<real_type>()*mat->density()), 1./3.);
    m -= vector3_type(d);
    M += vector3_type(d);

    vector3_type v((M-m)/size);
    v = ceil(v);

    if(phi.I()<v(0) ||  phi.J()<v(1)  || phi.K()<v(2))
      phi.create( m ,M
      , static_cast<math_types::index_type>(v[0])
      , static_cast<math_types::index_type>(v[1])
      , static_cast<math_types::index_type>(v[2])
      ); // TODO 2005-08-28 KE: hmmm, this should be pre-allocated!!!

    OpenTissue::grid::Grid<double,math_types>::index_iterator phi_begin = phi.begin();
    OpenTissue::grid::Grid<double,math_types>::index_iterator phi_end = phi.end();
    for(OpenTissue::grid::Grid<double,math_types>::index_iterator phi_ = phi_begin; phi_ != phi_end; ++phi_)
      *phi_ = 0.;

    /*
    SPHTypes::particle_container::const_iterator par = pars.begin();
    SPHTypes::particle_container::const_iterator par_end = pars.end();
    for (;par!=par_end;++par)
    {
    const vector3_type& n = par->normal();
    const SPHTypes::real_type l = n*n;
    if (l >= mat->threshold() || l < 0.0025)
    {
    const vector3_type &r = par->position();
    unsigned int i = static_cast<unsigned int>(std::floor( (r(0) - phi.min_coord(0)) / phi.dx() ));
    unsigned int j = static_cast<unsigned int>(std::floor( (r(1) - phi.min_coord(1)) / phi.dy() ));
    unsigned int k = static_cast<unsigned int>(std::floor( (r(2) - phi.min_coord(2)) / phi.dz() ));
    phi(  i,  j,k) = 100;
    phi(  i,j+1,k) = 100;
    phi(i+1,  j,k) = 100;
    phi(i+1,j+1,k) = 100;
    phi(  i,  j,k+1) = 100;
    phi(  i,j+1,k+1) = 100;
    phi(i+1,  j,k+1) = 100;
    phi(i+1,j+1,k+1) = 100;
    }
    }
    */

    vector3_type coord;
    for(OpenTissue::grid::Grid<double,math_types>::index_iterator phi_ = phi_begin; phi_ != phi_end; ++phi_)
    {
      //    if(*phi_>1.)
      {
        OpenTissue::grid::idx2coord(phi_,coord);
        *phi_ = sph->isoValue(coord);//0.35 - sph->isoValue(pos);
      }
    }


    //OpenTissue::mesh::isosurface(phi,0.0,surface);
    OpenTissue::mesh::smooth_isosurface(phi,0.6,surface,1,true);

    //  OpenTissue::polymesh::PolyMesh<>::vertex_iterator end = surface.vertex_end();
    //  for (OpenTissue::polymesh::PolyMesh<>::vertex_iterator vtx = surface.vertex_begin(); vtx != end; ++vtx)
    //    vtx->m_color = OpenTissue::polymesh::PolyMesh<>::vector3_type(mat->red(), mat->green(), mat->blue());

    glColor3d(mat->red(), mat->green(), mat->blue());
    //  glEnableClientState(GL_VERTEX_ARRAY);
    //  glEnableClientState(GL_NORMAL_ARRAY);

    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    OpenTissue::gl::DrawMesh(surface, GL_POLYGON, false, true, false);
    //  OpenTissue::MeshDrawArray<PolyMesh<> > dm(surface);
    //  dm();
    //OpenTissue::gl::DrawMesh(surface);

    //char *vertexArray = (char *) surface.getVertexArray();
    //glVertexPointer(3, GL_FLOAT, sizeof(ISvertex), vertexArray);
    //glNormalPointer(   GL_FLOAT, sizeof(ISvertex), vertexArray + sizeof(ISvec3));
    //glDrawElements(GL_TRIANGLES, surface.getIndexCount(), GL_UNSIGNED_INT, surface.getIndexArray());

    //  glDisableClientState(GL_NORMAL_ARRAY);
    //  glDisableClientState(GL_VERTEX_ARRAY);

    // draw bb
#if 0
    glDisable(GL_LIGHTING);
    glColor3d(0,0,0);
    glBegin(GL_LINES);
    glVertex3d(m[0], m[1], m[2]);
    glVertex3d(M[0], m[1], m[2]);
    glVertex3d(M[0], m[1], m[2]);
    glVertex3d(M[0], M[1], m[2]);
    glVertex3d(M[0], M[1], m[2]);
    glVertex3d(m[0], M[1], m[2]);
    glVertex3d(m[0], M[1], m[2]);
    glVertex3d(m[0], m[1], m[2]);
    glVertex3d(m[0], m[1], M[2]);
    glVertex3d(M[0], m[1], M[2]);
    glVertex3d(M[0], m[1], M[2]);
    glVertex3d(M[0], M[1], M[2]);
    glVertex3d(M[0], M[1], M[2]);
    glVertex3d(m[0], M[1], M[2]);
    glVertex3d(m[0], M[1], M[2]);
    glVertex3d(m[0], m[1], M[2]);
    glVertex3d(m[0], m[1], m[2]);
    glVertex3d(m[0], m[1], M[2]);
    glVertex3d(M[0], m[1], m[2]);
    glVertex3d(M[0], m[1], M[2]);
    glVertex3d(M[0], M[1], m[2]);
    glVertex3d(M[0], M[1], M[2]);
    glVertex3d(m[0], M[1], m[2]);
    glVertex3d(m[0], M[1], M[2]);
    glEnd();
#endif
    /*
    glEnable(GL_LIGHTING);
    for (SPHTypes::real_type k = m[2]; k < M[2]; k += size)
    for (SPHTypes::real_type j = m[1]; j <= M[1]; j += size)
    for (SPHTypes::real_type i = m[0]; i <= M[0]; i += size) {
    vector3_type test(i+.5*size, j+.5*size, k+.5*size);
    if ((0.35-sph->isoValue(test)) <= 0) {
    glBegin(GL_QUADS);
    glColor3d(1,0,0);
    glNormal3d(1, 0, 0);
    glVertex3d(i, j, k);
    glVertex3d(i, j+size, k);
    glVertex3d(i, j+size, k+size);
    glVertex3d(i, j, k+size);
    glColor3d(0,1,0);
    glNormal3d(0, 1, 0);
    glVertex3d(i, j, k);
    glVertex3d(i+size, j, k);
    glVertex3d(i+size, j, k+size);
    glVertex3d(i, j, k+size);
    glColor3d(0,0,1);
    glNormal3d(0, 0, 1);
    glVertex3d(i, j, k);
    glVertex3d(i+size, j, k);
    glVertex3d(i+size, j+size, k);
    glVertex3d(i, j+size, k);
    glEnd();
    }
    }
    */
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_CULL_FACE);
  }


  void addCollisionObject(Volume* obj)
  {
    object = obj;
    if (sph) {
      if (object == &capsule)
        sph->collisionSystem().addContainer(icapsule);
      else if (object == &sphere)
        sph->collisionSystem().addContainer(isphere);
      else if (object == &box)
        sph->collisionSystem().addContainer(ibox);
      else if (object == &DBb1)
        sph->collisionSystem().addContainer(iDBb1);
      else if (object == &DBb2)
        sph->collisionSystem().addContainer(iDBb2);
    }
  }


  template<typename MaterialPolicy>
  bool createFluid(size_t const & particles)
  {
    delete material;
    delete sph;

    material = new MaterialPolicy;
    material->particles() = particles;
    material->particle_mass(material->particle_mass());

    const SPHTypes::real_type  x = material->kernel_particles();         //--- Average number of particles inside kernel...

    material->threshold() = material->density()/x;

    Radius = material->radius(x);
    vector3_type gravity(0,0,-9.82);

    sph = new DefaultSystem;

    if (!sph->create(
      *static_cast< MaterialPolicy* >(material), gravity)
      )  // material, gravity
      return false;

    if (!sph->initHashing(2u*particles, Radius))  // hash table size, cell spacing (AABBs)
      return false;

    addCollisionObject(object);
    //  sph->collisionSystem().addContainer(isphere); object = &sphere;
    //  sph->collisionSystem().addObstacle(isphere2);
    //  sph->collisionSystem().addContainer(icapsule); object = &capsule;
    //  sph->collisionSystem().addObstacle(iplane);
    //  sph->collisionSystem().addObstacle(ibox); object = &box;
    //  sph->collisionSystem().addContainer(ibox); object = &box;
    //  sph->collisionSystem().addContainer(iDBb1); object = &DBb1;
    //  sph->collisionSystem().addContainer(iDBb2); object = &DBb2;

    if (use_emitter) 
    {
      emitter = &emitter2;
      emitter->batch() = 5;//7
      emitter->rate() = 2;

      if (!sph->init(emitter2, particles))
        return false;
    }
    else 
    {
      emitter = NULL;
      typedef std::vector<vector3_type> vectors;
      vectors positions, velocities;

      const SPHTypes::real_type dist = 1./45.;
      const SPHTypes::real_type off = -.14;
      size_t k = 1, j = 1, i = 1, p = particles;
      while (p--) 
      {
        positions.push_back(vector3_type(off+i*dist, off+j*dist, off+k*dist));
        velocities.push_back(vector3_type(0.,0.,0.));
        if (++i > 12) 
        {
          i = 1;
          if (++j > 12) 
          {
            j = 1;
            ++k;
          }
        }
      }
      if (!sph->init( positions.begin()
        , positions.end()
        , velocities.begin()
        , velocities.end())
        )
        return false;
    }
    return true;
  }


  /**
  * 2007-05-21 kenny: what is wrong with OpenTissue::gl::screen2object()?
  */
  void ScreenToWorld(double& xw, double& yw, double& zw, const int xs, const int ys)
  {
    GLdouble projMatrix[16];
    GLdouble modelViewMatrix[16];
    GLint viewPort[4];

    //  glMatrixMode(GL_MODELVIEW);
    //  glLoadIdentity();
    //  gluLookAt( eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz );
    //  glMultMatrixd( trackball.get_gl_current_rotation() );
    //  glRotatef(-90,1,0,0);

    glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv(GL_VIEWPORT, viewPort);

    gluUnProject(xs, viewPort[3]-ys, 1, modelViewMatrix, projMatrix, viewPort, &xw, &yw, &zw);
  }

public:

  char const * do_get_title() const { return "Smoothed Particle Hydrodynamics Demo Application"; }

  void do_display()
  {

    if (obstacles) 
    {
      glEnable(GL_COLOR_MATERIAL);
      glEnable(GL_LIGHTING);
      glColor3d(0.5, 0.0, 0.3);
      bool const wireframe = 1 == obstacles;
      if (CapsuleObj::id() == object->class_id())
        OpenTissue::gl::DrawCapsule(*static_cast<CapsuleObj*>(object), wireframe);
      else if (SphereObj::id() == object->class_id())
        OpenTissue::gl::DrawSphere(*static_cast<SphereObj*>(object), wireframe);
      else if (BoxObj::id() == object->class_id())
        OpenTissue::gl::DrawOBB(*static_cast<BoxObj*>(object), wireframe);
      glDisable(GL_LIGHTING);
    }


    if (use_emitter && emitter) 
    {
      glDisable(GL_COLOR_MATERIAL);
      glDisable(GL_LIGHTING);
      if (emitter->active()) 
      {
        if (emitter->running())
          glColor3d(0.1, 0.9, 0.1);
        else
          glColor3d(0.9, 0.1, 0.1);
      }
      else
        glColor3d(0.2, 0.2, 0.2);
      const vector3_type& c = emitter->center();
      const SPHTypes::real_type x = .03;

      glBegin(GL_LINES);
      glVertex3d(c(0)-x,c(1)-x,c(2)-x);
      glVertex3d(c(0)+x,c(1)-x,c(2)-x);

      glVertex3d(c(0)+x,c(1)-x,c(2)-x);
      glVertex3d(c(0)+x,c(1)+x,c(2)-x);

      glVertex3d(c(0)+x,c(1)+x,c(2)-x);
      glVertex3d(c(0)-x,c(1)+x,c(2)-x);

      glVertex3d(c(0)-x,c(1)+x,c(2)-x);
      glVertex3d(c(0)-x,c(1)-x,c(2)-x);

      glVertex3d(c(0)-x,c(1)-x,c(2)+x);
      glVertex3d(c(0)+x,c(1)-x,c(2)+x);

      glVertex3d(c(0)+x,c(1)-x,c(2)+x);
      glVertex3d(c(0)+x,c(1)+x,c(2)+x);

      glVertex3d(c(0)+x,c(1)+x,c(2)+x);
      glVertex3d(c(0)-x,c(1)+x,c(2)+x);

      glVertex3d(c(0)-x,c(1)+x,c(2)+x);
      glVertex3d(c(0)-x,c(1)-x,c(2)+x);

      glVertex3d(c(0)-x,c(1)-x,c(2)-x);
      glVertex3d(c(0)-x,c(1)-x,c(2)+x);

      glVertex3d(c(0)+x,c(1)-x,c(2)-x);
      glVertex3d(c(0)+x,c(1)-x,c(2)+x);

      glVertex3d(c(0)+x,c(1)+x,c(2)-x);
      glVertex3d(c(0)+x,c(1)+x,c(2)+x);

      glVertex3d(c(0)-x,c(1)+x,c(2)-x);
      glVertex3d(c(0)-x,c(1)+x,c(2)+x);
      glEnd();
    }


    const DefaultSystem::fluid_material* mat = sph?sph->material():NULL;
    if (sph) 
    {

      if (render)
        renderSurface(mat, 0.02);
      else
        renderParticles(mat);


      fps.frame(); // probe both sim + vis
      if (osd) 
      {
        glDisable(GL_LIGHTING);
        std::stringstream ss;
        ss << "FPS: " /*<< setw(4)*/ << fps();
        putText(this->width(), this->height()-30, 0.15, 0.0, 0.05, ss.str());
        std::ostringstream ost;
        ost << "Material: " << (mat?mat->name():"N/A");
        putText(16, this->height()-30, 0.15, 0.0, 0.05, ost.str());
        ost.str("");
        ost << "Particle Mass [kg]: " << (mat?mat->particle_mass():0.);
        putText(16, this->height()-50, 0.15, 0.0, 0.05, ost.str());
        ost.str("");
        ost << "Volume [m3]: " << (mat?mat->volume():0.);
        putText(16, this->height()-70, 0.15, 0.0, 0.05, ost.str());
        ost.str("");
        ost << "Particles: " << (mat?mat->particles():0.);
        putText(16, this->height()-90, 0.15, 0.0, 0.05, ost.str());
      }  
    }
  }

  void do_action(unsigned char choice)
  {
    using std::cos;
    using std::sin;

    switch (choice)
    {
    case 'n':
    case 'N':
      draw_normals = !draw_normals;
      break;
    case 'v':
    case 'V':
      draw_velocity = !draw_velocity;
      break;
    case 'w':
    case 'W':
      waves = !waves;
      break;
    case 's':
    case 'S':
      draw_surface = !draw_surface;
      break;
    case 'i':
    case 'I':
      draw_inside = !draw_inside;
      break;
    case 'o':
    case 'O':
      osd = !osd;
      break;
    case 'r':
    case 'R':
      render = !render;
      break;
    case 'c':
      if (sph) sph->collisionSystem().clear();
      if (object == &capsule)
        addCollisionObject(&sphere);
      else if (object == &sphere)
        addCollisionObject(&box);
      else
        addCollisionObject(&capsule);
      break;
    case 'C':
      if (++obstacles > 2)
        obstacles = 0;
      break;
    case 'E':
      use_emitter = !use_emitter;
      break;
    case 'e':
      if (!emitter) break;
      if (emitter->running())
        emitter->stop();
      else
        emitter->start();
      break;
    case '1':
      // create small water
      createFluid<WaterMaterial>(500);
      break;
    case '2':
      // create medium water
      createFluid<WaterMaterial>(1250);//1500
      break;
    case '3':
      // create medium water
      createFluid<WaterMaterial>(2250);
      break;
    case '4':
      // create large water
      createFluid<WaterMaterial>(4400);
      break;
    case '5':
      // create small mucus
      createFluid<MucusMaterial>(500);
      break;
    case '6':
      // create medium mucus
      createFluid<MucusMaterial>(1250);//1500
      break;
    case '7':
      // create medium mucus
      createFluid<MucusMaterial>(2250);
      break;
    case '8':
      // create large mucus
      createFluid<MucusMaterial>(4400);
      break;
    case '0':
      // create test steam
      createFluid<SteamMaterial>(2000);
      break;
    case '.':
      {
        const double a = to_radians<double>(45.), b = to_radians<double>(0.), c = to_radians<double>(0.);
        object->rotate(Volume::matrix3x3_type(1,0,0, 0,cos(a),sin(a), 0,-sin(a),cos(a)));
        object->rotate(Volume::matrix3x3_type(cos(b),0,-sin(b), 0,1,0, sin(b),0,cos(b)));
        object->rotate(Volume::matrix3x3_type(cos(c),sin(c),0, -sin(c),cos(c),0, 0,0,1));
        break;
      }
    case ',':
      {
        const double a = to_radians<double>(-2.), b = to_radians<double>(-2.), c = -2*to_radians<double>(-2.);
        object->rotate(Volume::matrix3x3_type(1,0,0, 0,cos(a),sin(a), 0,-sin(a),cos(a)));
        object->rotate(Volume::matrix3x3_type(cos(b),0,-sin(b), 0,1,0, sin(b),0,cos(b)));
        object->rotate(Volume::matrix3x3_type(cos(c),sin(c),0, -sin(c),cos(c),0, 0,0,1));
        break;
      }
    case '+':
      if (sph) 
      {
        sph->collisionSystem().clear();
        sph->collisionSystem().addContainer(iDBb2); object = &DBb2;
      }
      break;
    case '-':
      if (sph) 
      {
        sph->collisionSystem().clear();
        sph->collisionSystem().addContainer(iDBb1); object = &DBb1;
      }
      break;
    default:
      break;
    }
  }

  void do_init_right_click_menu(int main_menu, void menu(int entry))
  {
    int toggles = glutCreateMenu(menu);
    glutAddMenuEntry("On Screen Display [h]", 'h');
    glutAddMenuEntry("Render Surface [r]", 'r');
    glutAddMenuEntry("Surface Particles [s]", 's');
    glutAddMenuEntry("Intrinsic Particles [i]", 'i');
    glutAddMenuEntry("Surface Normals [n]", 'n');
    glutAddMenuEntry("Velocities [v]", 'v');
    glutAddMenuEntry("View Collision Objects [C]", 'C');
    glutAddMenuEntry("Change Collision Objects [c]", 'c');
    glutAddMenuEntry("Emitter [E]", 'E');
    glutAddMenuEntry("Start/Stop Emitter [e]", 'e');
    glutAddMenuEntry("Dam-Break: Init Dam [-]", '-');
    glutAddMenuEntry("Dam-Break: Break Dam [+]", '+');
    glutAddMenuEntry("Start/Stop Box Waves [w]", 'w');

    int materials = glutCreateMenu(menu);
    glutAddMenuEntry("Water  (500) [1]", '1');
    glutAddMenuEntry("Water (1250) [2]", '2');
    glutAddMenuEntry("Water (2250) [3]", '3');
    glutAddMenuEntry("Water (4400) [4]", '4');
    glutAddMenuEntry("Mucus  (500) [5]", '5');
    glutAddMenuEntry("Mucus (1250) [6]", '6');
    glutAddMenuEntry("Mucus (2250) [7]", '7');
    glutAddMenuEntry("Mucus (4400) [8]", '8');
    glutAddMenuEntry("Steam (test) [0]", '0');

    glutSetMenu(main_menu);
    glutAddSubMenu("toggles", toggles);
    glutAddSubMenu("materials", materials);
  }

  void do_init()
  {
    this->camera().move(95);
    this->zoom_sensitivity() = 0.02;
    this->pan_sensitivity() = 0.01;
  }

  void do_run()
  {
    if (emitter)
    {
      emitter->execute();
    }
    if (waves) 
    {
      vector3_type ext = DBb2.ext();
      vector3_type cen = DBb2.center();
      static double t = 0;
      using std::fabs;
      const double w = 0.1*fabs(sin(t));
      ext(0) = 0.5+w; t += 0.05;
      cen(0) = 0.3125+w;
      DBb2.set(cen, DBb2.orientation(), ext);
    }
    //  for (int n = 0; n < 10; ++n) sph->simulate();
    sph->simulate();
  }

  void do_shutdown(){}

  void mouse_down(double cur_x,double cur_y,bool shift,bool ctrl, bool alt,bool left,bool middle,bool right) 
  {
    if( !bScaleObstacle && middle && ctrl)
    {
      yScale = boost::numeric_cast<int>(cur_y);
      bScaleObstacle = true;
    }
    else if( !bPanObstacle && left && shift && ctrl)
    {
      bPanObstacle = true;
      ScreenToWorld(xPan, yPan, zPan, boost::numeric_cast<int>(cur_x), boost::numeric_cast<int>(cur_y));
    }
    else
    {
      OpenTissue::graphics::PerspectiveViewApplication::mouse_down(cur_x,cur_y,shift, ctrl, alt, left, middle, right);
    }
  }

  void mouse_move(double cur_x,double cur_y) 
  {
    
    if (bPanObstacle) 
    {
      double x, y, z;
      ScreenToWorld(x, y, z, boost::numeric_cast<int>(cur_x), boost::numeric_cast<int>(cur_y));
      object->translate(this->pan_sensitivity()*vector3_type(x-xPan,y-yPan,z-zPan));
      xPan = x;
      yPan = y;
      zPan = z;
    }
    else if (bScaleObstacle) 
    {
      const GLdouble scale = cur_y - yScale;
      if (scale > 0)
        object->scale(1.02);
      else if (scale < 0)
        object->scale(0.98);
      yScale = boost::numeric_cast<int>(cur_y);
    }
    else
    {
      OpenTissue::graphics::PerspectiveViewApplication::mouse_move(cur_x,cur_y);
    }
  }

  void mouse_up(double cur_x,double cur_y,bool shift,bool ctrl, bool alt, bool left,bool middle,bool right) 
  {
    if( bScaleObstacle)
    {
      yScale = boost::numeric_cast<int>(cur_y);
      bScaleObstacle = false;
    }
    else if(  bPanObstacle )
    {
      ScreenToWorld(xPan, yPan, zPan, boost::numeric_cast<int>(cur_x), boost::numeric_cast<int>(cur_y));
      bPanObstacle = false;
    }
    else
    {
      OpenTissue::graphics::PerspectiveViewApplication::mouse_up(cur_x,cur_y,shift, ctrl, alt, left, middle, right);
    }
  }

};

OpenTissue::graphics::instance_pointer init_glut_application(int argc, char **argv)
{
  OpenTissue::graphics::instance_pointer instance;
  instance.reset( new Application() );
  return instance;
}
