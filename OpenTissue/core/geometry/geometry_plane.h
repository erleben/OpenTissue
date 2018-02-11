#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PLANE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PLANE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_base_shape.h>
#include <OpenTissue/core/geometry/geometry_compute_plane_aabb.h>
#include <OpenTissue/core/function/function_signed_distance_function.h>
#include <OpenTissue/utility/utility_class_id.h>
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/math/math_precision.h>

#include <cmath>
#include <iosfwd>

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * A Plane.
    * The Plane consist of all points x, where
    *
    *                n*x-d = 0
    */
    template< typename math_types_ >
    class Plane
      : public BaseShape< math_types_ >
      , public function::SignedDistanceFunction< math_types_ >
      , public OpenTissue::utility::ClassID< Plane<math_types_> >
    {
    public:

      typedef          math_types_                   math_types;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::matrix3x3_type    matrix3x3_type;

    protected:

      typedef typename math_types::value_traits      value_traits;
      typedef typename math_types::real_type         real_type;
      typedef          Plane<math_types>             plane_type;

    public:

      vector3_type m_n;    ///< The Plane Normal.
      real_type    m_d;    ///< The offset of the plane (in the direction
      ///< of the normal) from the origin of the
      ///< coordinate frame.
    public:

      vector3_type const & n() const { return m_n; }
      vector3_type       & n()       { return m_n; }
      real_type    const & w() const { return m_d; }
      real_type          & w()       { return m_d; }

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Plane<math_types_> >::class_id(); }

      /**
      * Default constructor.
      * Sets up a plane such that:
      *
      * n = 0 and d =0
      *
      * This means that all points no matter what their
      * poisition would be lines in the plane.
      */
      Plane()
        : m_n(value_traits::zero(),value_traits::zero(),value_traits::zero())
        , m_d(value_traits::zero())
      {}

      /**
      * Special constructor.
      * Works in the same way as the corresponding assignment method.
      *
      * @param p1   A point in the plane.
      * @param p2   A point in the plane.
      * @param p3   A point in the plane.
      */
      explicit Plane(vector3_type const & p1,vector3_type const & p2,vector3_type const & p3)    {      set(p1,p2,p3);    }

      /**
      * Copy constructor.
      * Works in the same way as the corresponding assignment method.
      *
      * @param Eq  Another plane equation.
      */
      Plane(Plane const & p)    {      set(p);    }

      /**
      * Special Constructor.
      *
      * @param n  The direction of the normal vector of this plane.
      * @param d  The orthogonal distance from origo.
      */
      explicit Plane(vector3_type const & n_val, real_type const & d_val){      set(n_val,d_val);    }

      /**
      * Special constructor.
      * Works in the same way as the corresponding assignment method.
      *
      * @param ndir  The direction of the normal vector of this plane.
      * @param p     A point in the plane.
      */
      explicit Plane(vector3_type const & ndir, vector3_type const & p)
      {
        set(ndir,p);
      }

    public:

      bool is_equal(plane_type const & p)const
      {
        // TODO: Comparing floats with ==
        if(m_n==p.m_n && m_d==p.m_d)
          return true;
        return false;
      }

      bool isNearlyEqual(plane_type const & p)const
      {
        // TODO: Check that this is actually what we want!
        real_type epsilon = math::working_precision<real_type>();
        if(m_n.is_equal(p.m_n,epsilon)&& std::fabs(m_d-p.m_d)<epsilon)
          return true;
        return false;
      }

      /**
      * Assignment Method.
      * Assigns the values of the specified plane
      * equation to this plane equation.
      *
      * @param Eq  Another plane equation.
      */
      void set(plane_type const & p)
      {
        m_n = p.m_n;
        m_d = p.m_d;
      }

      /**
      * Assignment Method.
      *
      * @param n  The direction of the normal vector of this plane.
      * @param d  The orthogonal distance from origo.
      */
      void set(vector3_type const & n_val,real_type const & d_val)
      {
        m_n = unit(n_val);
        m_d = d_val;
      }

      /**
      * Assignment Method.
      * Assigns values to this plane in such a way, that
      * the plane will have a unit vector normal in the
      * same direction as the first argument, and in
      * such a way that the second argument lies in
      * the plane represented by this plane equation.
      *
      * @param ndir  The direction of the normal vector of this plane.
      * @param p     A point in the plane.
      */
      void set(vector3_type const & ndir,vector3_type const & p)
      {
        m_n = unit(ndir);
        m_d = m_n *p;
      }

      /**
      * Assignment Method.
      * Assigns values to this plane in such a way that all the three
      * specified points lie in the plane.
      *
      * VERY IMPORTANT: The three points must all be different and not collinear.
      *
      * @param p1   A point in the plane.
      * @param p2   A point in the plane.
      * @param p3   A point in the plane.
      */
      void set(vector3_type const & p1,vector3_type const & p2,vector3_type const & p3)
      {
        vector3_type u1,u2,u1Xu2;
        u2 = p3 - p2;
        u1 = p2 - p1;
        u1Xu2 = cross(u1,u2);
        m_n = unit(u1Xu2);
        m_d = m_n * p1;
      }

      /**
      * This method computes the plane equation from three given
      * points which lies in the plane.
      *
      * @param c1    First point of the plane.
      * @param c2    Second point of the plane.
      * @param c3    Third point of the plane.
      */
      void set(real_type * c1,real_type * c2,real_type * c3)
      {
        vector3_type p1(c1[0],c1[1],c1[2]);
        vector3_type p2(c2[0],c2[1],c2[2]);
        vector3_type p3(c3[0],c3[1],c3[2]);
        set(p1,p2,p3);
      }

      /**
      * Coplanar Test.
      * This method test if the specified plane and this
      * plane are coplanar. That is whatever their normals
      * point in the same direction.
      */
      bool is_coplanar(plane_type const& p) const
      {
        return (m_n == p.m_n);
      }

      /**
      * This method computes two orthogonal vectors, which
      * span the plane.
      *
      * These vectors are usefull when one wants to project
      * points onto the plane.
      *
      * @param tx   Upon return this vector holds the x-axe direction in the plane.
      * @param ty   Upon return this vector holds the y-axe direction in the plane.
      */
      void compute_plane_vectors(vector3_type & tx,vector3_type & ty) const
      {
        //--- Use ty as temporary storage for computing a projection axe (which is temporarily stored in tx)
        vector3_type i(value_traits::one(), value_traits::zero(),value_traits::zero());
        vector3_type j(value_traits::zero(),value_traits::one(), value_traits::zero());
        vector3_type k(value_traits::zero(),value_traits::zero(),value_traits::one());

        ty = cross(m_n,i);
        if(!is_zero(ty) )
        {
          tx = i;
        }
        else
        {
          ty = cross(m_n,j);
          if( !is_zero(ty) )
          {
            tx = j;
          }
          else
          {
            ty = cross(m_n,k);
            if( !is_zero(ty) )
            {
              tx = k;
            }
            else
            {
              std::cout << "plane::compute_plane_vectors(): Could not find non-parallel vector to n" << std::endl;
              return;
            }
          }
        }
        //--- Now we can compute ty
        ty = unit(cross(m_n,tx));
        //--- And finally tx
        tx = unit(cross(ty,m_n));
      }

      /**
      * Retrieves the shortest distance from the
      * specified point to this plane.
      *
      * @param p          A point
      * @return           The shortest distance from
      *                   the specified point to this
      *                   plane.
      */
      real_type get_distance(vector3_type const & p)const
      {
        using std::fabs;

        return fabs(m_n*p - m_d);
      }

      /**
      * Retrieves the shortest distance with sign from the
      * specified point to this plane.
      *
      * If the point lies on the side of the plane where the normal
      * is pointing the sign would be positive. This side of the
      * plane is known as the frontside. If the point was on the
      * backside the sign would have been negative. The final possibility
      * is that the point lies in the plane. In this case the reutrn
      * value is zero.
      *
      * @param p          A point
      * @return           The shortest signed distance from the
      *                   specified point to this plane.
      */
      real_type signed_distance(vector3_type const & p)const
      {
        return m_n*p-m_d;
      }

      void compute_surface_points( std::vector<vector3_type> & /*points*/) const
      {
      }

      vector3_type get_support_point(vector3_type const & /*v*/) const
      {
        return vector3_type();
      }

      void scale(real_type const & /*s*/)
      {
        assert("Plane::scale doesn't make any sense!");
      }

      /**
      * Rotation of Plane.
      * The transformation rule is easy to derive, let prime
      * values denote the new value of this plane equation
      * and non-prime the current values.
      *
      * d  = n*p
      * n' = R*n
      * d' = Rn*(t+Rp) |  where p is old point in plane
      * d' = (Rn)*t + (Rn)*(Rp)
      * d' = n'*t + d
      *
      * @param R  The rotation of the plane around origo.
      */
      void rotate(matrix3x3_type const & R)
      {
        m_n = R * m_n;
      }

      /**
      * Translation of Plane.
      *
      * @param T  The translation of the plane.
      */
      void translate(vector3_type const & T)
      {
        m_d += m_n * T;
      }

      /**
      * Flip Plane.
      */
      void flip()
      {
        m_n = - m_n;
        m_d = -m_d;
      }

      /**
      * Project Point onto Plane.
      *
      * @param point    The point that should be projected.
      * @return         The projected point
      */
      vector3_type project(vector3_type const & point) const
      {
        real_type h = m_n * point - m_d;
        vector3_type tmp = point - m_n*h;
        return tmp;
      }

    public:

      real_type evaluate(vector3_type const & x) const
      {
        return signed_distance(x);
      }

      vector3_type gradient(vector3_type const & /* x */) const
      {
        return m_n;
      }

      vector3_type normal(vector3_type const & /* x */) const
      {
        return unit(m_n);
      }

      /**
      * Compute Bounding Box.
      * This method computes an axis aligned bounding
      * box (AABB) that encloses the geometry.
      *
      * @param r           The position of the model frame (i.e the coordinate frame the geometry lives in).
      * @param R           The orientation of the model frame (i.e the coordinate frame the geometry lives in).
      * @param min_coord   Upon return holds the minimum corner of the box.
      * @param max_coord   Upon return holds the maximum corner of the box.
      *
      */
      void compute_collision_aabb(
          vector3_type const & /*r*/
        , matrix3x3_type const & /*R*/
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {
        OpenTissue::geometry::compute_plane_aabb(this->n(), this->w(), min_coord, max_coord);
      }

    };


    /**
    *
    * Factory method making it easier to create planes on the fly.
    *
    * @param n     The outward normal of the wanted plane.
    * @param w     The distance from origo along the normal vector.
    *
    * @return      The newly created plane.
    */
    template<typename T>
    inline Plane< typename math::BasicMathTypes< T, size_t> > 
      make_plane(math::Vector3<T> const & n, T const & w)
    {
      typedef typename math::BasicMathTypes<T,size_t>  math_types;
      typedef          Plane< math_types >             plane_type;
      return plane_type(n,w);
    }


    template<typename T>
    std::ostream & operator<< (std::ostream & o, Plane<T> const & p)
    {
      o << "["
        << p.n()(0)
        << ","
        << p.n()(1)
        << ","
        << p.n()(2)
        << ","
        << p.w()
        << "]";
      return o;
    }

    template<typename T>
    std::istream & operator>>(std::istream & i, Plane<T> & p)
    {
      char dummy;
      i >> dummy;
      i >> p.n()(0);
      i >> dummy;
      i >> p.n()(1);
      i >> dummy;
      i >> p.n()(2);
      i >> dummy;
      i >> p.w();
      i >> dummy;
      return i;
    }

  }  // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PLANE_H
#endif
