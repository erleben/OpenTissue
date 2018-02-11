#ifndef OPENTISSUE_DYNAMICS_SPH_COLLISION_POLICIES_SPH_IMPLICIT_PRIMITIVES_POINTS_H
#define OPENTISSUE_DYNAMICS_SPH_COLLISION_POLICIES_SPH_IMPLICIT_PRIMITIVES_POINTS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/dynamics/sph/collision/sph_collision_type.h>
#include <OpenTissue/core/math/math_constants.h>
#include <OpenTissue/core/math/math_functions.h>

#include <map>


namespace OpenTissue
{
  namespace sph
  {

    template <
      typename real_type_
      , typename vector_type
    >
    class ImplicitPrimitive
    {
    public:
      typedef real_type_    real_type;
      typedef vector_type  vector;

    public:
      virtual ~ImplicitPrimitive()
      {}

    public:
      virtual real_type F(const vector& p) const = 0;

      virtual void collisionInfo(vector& cp, vector& n, real_type& d, const vector& p) const = 0;

    };


    template <
      typename real_type_
      , typename vector_type
      , typename sphere_type
    >
    class ImplicitSpherePrimitive : public ImplicitPrimitive<real_type_, vector_type>
    {
    public:
      
      typedef ImplicitPrimitive<real_type_, vector_type>  base_type;
      typedef real_type_  real_type;
      typedef typename base_type::vector  vector;
      typedef sphere_type  sphere;

    public:
      ImplicitSpherePrimitive(const sphere& sphere) 
        : base_type()
        , m_sphere(sphere)
      {}

      ~ImplicitSpherePrimitive()
      {}

    public:
      real_type F(const vector& p) const
      {
        const vector tmp = p-m_sphere.center();
        return tmp*tmp - m_sphere.squared_radius();
      }

      void collisionInfo(vector& cp, vector& n, real_type& d, const vector& p) const
      {
        const vector tmp = p-m_sphere.center();
        const real_type depth = length(tmp)-m_sphere.radius();
        d = std::fabs(depth);
        n = normalize(tmp);
        cp = m_sphere.center()+n*m_sphere.radius();
        //      n *= depth>0?-1:1;
        n *= math::sgn(depth);
      }

    private:
      ImplicitSpherePrimitive& operator=(const ImplicitSpherePrimitive& rhs)
      {
      }

    private:
      const sphere&  m_sphere;

    };


    template <
      typename real_type_
      , typename vector_type
      , typename capsule_type
    >
    class ImplicitCapsulePrimitive : public ImplicitPrimitive<real_type_, vector_type>
    {
    public:
      typedef ImplicitPrimitive<real_type_, vector_type>  base_type;
      typedef real_type_  real_type;
      typedef typename base_type::vector  vector;
      typedef capsule_type  capsule;

    public:
      ImplicitCapsulePrimitive(const capsule& capsule) : base_type()
        , m_capsule(capsule)
      {}

      ~ImplicitCapsulePrimitive()
      {}

    public:
      real_type F(const vector& p) const
      {
        return m_capsule.evaluate(p);
      }

      void collisionInfo(vector& cp, vector& n, real_type& d, const vector& p) const
      {
        using std::fabs;
        d = fabs(m_capsule.signed_distance(p));
        n = m_capsule.normal(p);
        cp = m_capsule.closest_point_on_line_segment(p)+n*m_capsule.radius();
      }

    private:
      ImplicitCapsulePrimitive& operator=(const ImplicitCapsulePrimitive& rhs)
      {
      }

    private:
      const capsule&  m_capsule;

    };


    template <
      typename real_type_
      , typename vector_type
    >
    class ImplicitPlanePrimitive : public ImplicitPrimitive<real_type_, vector_type>
    {
    public:
      typedef ImplicitPrimitive<real_type_, vector_type>  base_type;
      typedef real_type_  real_type;
      typedef typename base_type::vector  vector;

    public:
      ImplicitPlanePrimitive(const vector& x0 = 0, const vector& n = 0) : base_type()
        , m_x0(x0)
        , m_n(n)
      {}

      ~ImplicitPlanePrimitive()
      {}

    public:

      const vector& point() const
      {
        return m_x0;
      }

      vector& point()
      {
        return m_x0;
      }

      const vector& normal() const
      {
        return m_n;
      }

      vector& normal()
      {
        return m_n;
      }

    public:
      real_type F(const vector& p) const
      {
        return m_n*(p-m_x0);
      }

      void collisionInfo(vector& cp, vector& n, real_type& d, const vector& p) const
      {
        const vector tmp = p-m_x0;
        n = normalize(m_n);
        d = std::fabs(tmp*n);
        cp = p+n*d;
      }

    private:
      vector  m_x0;
      vector  m_n;

    };

#if 1
    template <
      typename real_type_
      , typename vector_type
      , typename box_type
    >
    class ImplicitBoxPrimitive : public ImplicitPrimitive<real_type_, vector_type>
    {
    public:
      typedef ImplicitPrimitive<real_type_, vector_type>  base_type;
      typedef real_type_  real_type;
      typedef typename base_type::vector  vector;
      typedef box_type  box;

    public:
      ImplicitBoxPrimitive(const box& box) : base_type()
        , m_box(box)
      {}

      ~ImplicitBoxPrimitive()
      {}

    public:

      real_type F(const vector& p) const
      {
        const vector p_local = trans(m_box.orientation())*(p-m_box.center());
        const vector tmp = fabs(p_local)-m_box.ext();
        const real_type res = max_value(tmp);

        return res;
      }

      void collisionInfo(vector& cp, vector& n, real_type& d, const vector& p) const
      {
        const vector c = m_box.center();
        typename box::matrix3x3_type const& R = m_box.orientation();
        typename box::matrix3x3_type const Rt = trans(R);
        const vector& x = m_box.ext();

        /*
        vector cpl = n = Rt*(p-c);
        cpl = max(-x, min(cpl, x));
        cp = cpl-n;
        cpl = c + R*cpl;
        using std::fabs;
        const double E = 0.00000001;
        if (fabs(cp[0]) < E) cp[0] = 0;
        if (fabs(cp[1]) < E) cp[1] = 0;
        if (fabs(cp[2]) < E) cp[2] = 0;
        n = R*sgn(cp);
        n = unit(n);
        const vector m(n);
        cout << "diff: " << cpl-p << "  n: " << n << endl;
        */
        int coli = 0;
        cp = Rt*(p-c);
        n = vector(0);
        if (cp(0) > x(0)) {
          cp(0) = x(0);
          n -= Rt[0];
          coli++;
        }
        else if (cp(0) < -x(0)) {
          cp(0) = -x(0);
          n += Rt[0];
          coli++;
        }

        if (cp(1) > x(1)) {
          cp(1) = x(1);
          n -= Rt[1];
          coli++;
        }
        else if (cp(1) < -x(1)) {
          cp(1) = -x(1);
          n += Rt[1];
          coli++;
        }

        if (cp(2) > x(2)) {
          cp(2) = x(2);
          n -= Rt[2];
          coli++;
        }
        else if (cp(2) < -x(2)) {
          cp(2) = -x(2);
          n += Rt[2];
          coli++;
        }

        cp = c + R*cp;
        d = length(cp-p);
        n = unit(n);

        // if particle collides with more than one side of the box, then add a random displacement to cp
        // this will prevent particles from ending up with the same position, which cause mayhem to the simulation.
        if (coli > 1) {
          vector disp;
          random(disp, 0, 0.0001);
          disp(0) *= n(0);
          disp(1) *= n(1);
          disp(2) *= n(2);
          cp += disp;
        }
        /*
        cout << "diff: " << cp-p << "  n: " << n << endl;

        if (m != n)
        int a = 0;
        */
      }

    private:
      ImplicitBoxPrimitive& operator=(const ImplicitBoxPrimitive& rhs)
      {}

    private:
      const box&  m_box;

    };
#else
    template <
      typename real_type_
      , typename vector_type
    >
    class ImplicitBoxPrimitive : public ImplicitPrimitive<real_type_, vector_type>
    {
    public:
      typedef ImplicitPrimitive<real_type_, vector_type>  base_type;
      typedef ImplicitPlanePrimitive<real_type_, vector_type>  plane;
      typedef real_type_  real_type;
      typedef typename base_type::vector  vector;

    public:
      ImplicitBoxPrimitive(const vector& pMin = 0, const vector& pMax = 0)
        : m_pMin(pMin)
        , m_pMax(pMax)
      {
        setBox();
      }

    public:

      void setBox()
      {
        const vector center = m_pMin+0.5*(m_pMax-m_pMin);

        m_p[0].point() = vector(m_pMin[0], center[1], center[2]);
        m_p[0].normal() = vector(1., 0., 0.);

        m_p[1].point() = vector(m_pMax[0], center[1], center[2]);
        m_p[1].normal() = vector(-1., 0., 0.);

        m_p[2].point() = vector(center[0], m_pMin[1], center[2]);
        m_p[2].normal() = vector(0., 1., 0.);

        m_p[3].point() = vector(center[0], m_pMax[1], center[2]);
        m_p[3].normal() = vector(0., -1., 0.);

        m_p[4].point() = vector(center[0], center[1], m_pMin[2]);
        m_p[4].normal() = vector(0., 0., 1.);

        m_p[5].point() = vector(center[0], center[1], m_pMax[2]);
        m_p[5].normal() = vector(0., 0., -1.);
      }

      const vector& min_point() const
      {
        return m_pMin;
      }

      vector& min_point()
      {
        return m_pMin;
      }

      const vector& max_point() const
      {
        return m_pMax;
      }

      vector& max_point()
      {
        return m_pMax;
      }

    public:
      real_type F(const vector& p) const
      {
        using std::min;

        real_type res = math::detail::highest<real_type>();
        for (int n = 0; n < 6; ++n)
          res = min(res, m_p[n].F(p));
        return res;
      }

      void collisionInfo(vector& cp, vector& n, real_type& d, const vector& p) const
      {
        n = 0.;
        for (int i = 0; i < 6; ++i)
          if (m_p[i].F(p) < 0.)
            n += m_p[i].normal();
        cp = p;

        if (n[0] > 0.)
          cp[0] = m_pMin[0];
        else if (n[0] < 0.)
          cp[0] = m_pMax[0];

        if (n[1] > 0.)
          cp[1] = m_pMin[1];
        else if (n[1] < 0.)
          cp[1] = m_pMax[1];

        if (n[2] > 0.)
          cp[2] = m_pMin[2];
        else if (n[2] < 0.)
          cp[2] = m_pMax[2];

        n = unit(n);
        d = length(cp-p);
      }

    private:
      vector  m_pMin;
      vector  m_pMax;
      plane  m_p[6];

    };
#endif

    template <
      typename real_type_
      , typename vector_type
      , typename point_data_type
    >
    class ImplicitPrimitivesCollisionDetectionPolicy
    {
    public:
      typedef real_type_  real_type;
      typedef vector_type  vector;
      typedef point_data_type  point_data;
      typedef enum {CONTAINER, OBSTACLE} primitive_type;
      typedef CollisionType<real_type, vector>  collision_type;
      typedef ImplicitPrimitive<real_type, vector_type> implicit_primitive; // TODO: design flaw :(
      typedef std::map<const implicit_primitive*, primitive_type>  implicit_primitives;

    public:

      void clear()
      {
        m_primitives.clear();
      }

      template< typename implicit_ptimitive >
      void addObstacle(const implicit_ptimitive& obstacle)
      {
        m_primitives[&obstacle] = OBSTACLE;
      }

      template< typename implicit_ptimitive >
      void addContainer(const implicit_ptimitive& container)
      {
        m_primitives[&container] = CONTAINER;
      }

      void refresh()
      {
      }

      bool collision(collision_type& collision, const point_data& query)
      {
        typename implicit_primitives::const_iterator end = m_primitives.end();
        for (typename implicit_primitives::const_iterator ips = m_primitives.begin(); ips != end; ++ips) {
          const implicit_primitive* ip = ips->first;
          const real_type val = ip->F(query.position());
          if ((CONTAINER == ips->second && val > 0.) ||
            (OBSTACLE == ips->second && val < 0.))
          {
            ip->collisionInfo(collision.contact(), collision.normal(), collision.penetration(), query.position());
            return true;
          }
        }
        return false;
      }

    protected:

      implicit_primitives  m_primitives;

    };

  } // namespace sph
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_SPH_COLLISION_POLICIES_SPH_IMPLICIT_PRIMITIVES_POINTS_H
#endif
