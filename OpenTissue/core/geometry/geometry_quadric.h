#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_QUADRIC_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_QUADRIC_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <cmath>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Internal auxiliary class.
    *
    * For instance in homegeneous coordinates all points on the surface of an
    * ellipsoid must fulfill the equation
    *
    *          | A   B| |x|
    *  |x^T 1| | B^T C| |1|  = 0
    *
    * This is equivalent to the condition
    *
    *   x^T A2 x + 2 B2^T x + C2 = 0
    *
    * The left-hand-side (LHS) is a quadratic function in x, this class
    * helops representing the LHS of this equation.
    *
    *
    * Note that a non-symmetrical quadric can be made symmetrical by
    *
    *          | A   E| |x|
    *  |x^T 1| | D^T C| |1|  = 0   =>   x^T A x + x^T E + D^T x + C =   x^T A x + (E + D)^T x + C = 0
    *
    * Then define 
    *
    *   B = (E + D)/2
    *
    *
    */
    template<typename real_type_>
    class Quadric
    {
    public:

      typedef real_type_                             real_type;
      typedef OpenTissue::math::Vector3<real_type>   vector3_type;
      typedef OpenTissue::math::Matrix3x3<real_type> matrix3x3_type;
      typedef Quadric<real_type>                     quadric_type;

    public:

      matrix3x3_type  m_A;
      vector3_type    m_B;
      real_type       m_C;

    public:

      Quadric()
        : m_A(1,0,0,0,1,0,0,0,1)
        , m_B(0,0,0)
        , m_C(0)
      {}

      Quadric(matrix3x3_type const & A,vector3_type const & B,real_type const & C)
        : m_A(A)
        , m_B(B)
        , m_C(C)
      {}

    public:

      real_type operator()(vector3_type const & x){ return x*(m_A*x) + 2.0*m_B*x + m_C; }

      quadric_type & operator=(quadric_type const & q)
      {
        m_A = q.m_A;
        m_B = q.m_B;
        m_C = q.m_C;
        return (*this);
      }

      quadric_type & operator*(real_type const & s)
      {
        m_A *= s;
        m_B *= s;
        m_C *= s;
        return (*this);
      }

      quadric_type operator+(quadric_type const & q){ return quadric_type( m_A + q.m_A, m_B + q.m_B, m_C + q.m_C ); }

    public:

      /**
      * Set Quadric to a Sphere.
      *
      * @param radius   The radius of the sphere.
      * @param center   The center of the sphere.
      */
      void set_sphere(real_type const & radius, vector3_type const & center)
      {
        m_A = matrix3x3_type( 1,0,0, 0,1,0, 0,0,1);
        m_B = - center;
        m_C = center*center - radius*radius;
      }

      /**
      * Plane Product Quadric.
      *
      * Defining:
      *
      * (x - p)^T np * ( q - x)^T nq = 0          
      *
      * This means that x must lie either on plane (p,np) or (q,nq)... Note this
      * is signed distance of one plane multiplied by the negative signed distance
      * of the other plane.
      *
      * Retwritting using homegeneous coordintes
      *
      *          | A2   B2| |x|
      *  |x^T 1| | B2^T C2| |1|  = 0
      *
      * Which is equivalent to
      *
      *     x^T A2 x + 2 B2^T x + C2 = 0
      *
      * Straightforward computation then shows that
      *
      * A2   =    - np nq^T
      *
      *           | np0 nq*q + nq0 np*p |
      * 2 B2 =    | np1 nq*q + nq1 np*p |
      *           | np2 nq*q + nq2 np*p |
      *
      * C2 = - p^T np * q^T nq
      *
      * Observe that if the point x lies on opposite sides of the two planes
      * then the sign is positive. On the other hand if x lies on the same
      * side (front or back) of the two planes, then the sign is negative.
      */
      void set_plane_product(vector3_type const & p,vector3_type const & np,vector3_type const & q,vector3_type const & nq)
      {
        //--- A   =  -  np nq^T
        //--- 2 B =  np (nq^T q)  + nq (np^T p)
        //--- C   =  - p^T np * q^T nq
        m_A   = - outer_prod(np,nq);
        real_type nqq = nq*q;
        real_type npp = np*p;
        m_B   = (np*nqq + nq*npp)*.5;
        m_C   = - npp*nqq;

        //--- Unfortunately we might be in a mess if A is
        //--- not symmetric, however computing A = (A+A^T)/2 solves
        //--- the problem, the quadric is still the same quadric
        m_A(0,1) += m_A(1,0);
        m_A(0,1) *= .5;
        m_A(0,2) += m_A(2,0);
        m_A(0,2) *= .5;
        m_A(1,2) += m_A(2,1);
        m_A(1,2) *= .5;
        m_A(1,0) = m_A(0,1);
        m_A(2,0) = m_A(0,2);
        m_A(2,1) = m_A(1,2);
      }

      /**
      * Defines a quadric that represent an elliptical cylinder tube. The
      * argument defines a planar ellipsoid profile (cross-section) that
      * sweeps out the cylindrical tube.
      *
      * @param center   The center of the ellipsoid
      * @param axis0    The first axis of the ellipsoid.
      * @param axis1    The secod axis of the ellipsoid (orthogonal to the first).
      * @param radius0  The radius of the first axis.
      * @param radius1  The radius of the second axis.
      */
      void set_cylinder(
        vector3_type const & center
        , vector3_type const & axis0, real_type const & radius0
        , vector3_type const & axis1, real_type const & radius1
        )
      {
        matrix3x3_type A0  = outer_prod(axis0,axis0);
        matrix3x3_type A1  = outer_prod(axis1,axis1);
        real_type r0r0 = radius0*radius0;
        real_type r1r1 = radius1*radius1;
        m_A = r1r1*A0 + r0r0*A1;
        real_type ca0 = center*axis0;
        real_type ca1 = center*axis1;
        m_B = - (r1r1*ca0)*axis0 - (r0r0*ca1)*axis1;
        m_C = r1r1*ca0*ca0 + r0r0*ca1*ca1 - r0r0*r1r1;
      }

    };


    template<typename real_type,typename vector3_type>
    Quadric<real_type> make_sphere_quadric(real_type const & radius, vector3_type const & center)
    {
      Quadric<real_type> Q;
      Q.set_sphere(radius,center);
      return Q;
    }

    template<typename real_type,typename vector3_type>
    Quadric<real_type> make_cylinder_quadric(
      vector3_type const & center
      , vector3_type const & axis0
      , real_type    const & radius0
      , vector3_type const & axis1
      , real_type    const & radius1
      )
    {
      Quadric<real_type> Q;
      Q.set_cylinder(center,axis0,radius0,axis1,radius1);
      return Q;
    }

    template<typename vector3_type>
    Quadric<typename vector3_type::value_type> make_plane_product_quadric(vector3_type const & p,vector3_type const & np,vector3_type const & q,vector3_type const & nq)
    {
      Quadric<typename vector3_type::value_type> Q;
      Q.set_plane_product(p,np,q,nq);
      return Q;
    }

    /**
    * Test if Quadris is a valid ellipsoid
    *
    * @param Q  The quadric.
    *
    * @return  If the specified quadric can be transformed into a valid ellipsoid
    *          then the return value is true otherwise it is false.
    */
    template<typename quadric_type>
    bool is_valid_ellipsoid(quadric_type const & Q)
    {
      typename quadric_type::matrix3x3_type V;
      typename quadric_type::vector3_type   d;
      math::eigen(Q.m_A,V,d);

      if(d(0)<0)
        return false;
      if(d(1)<0)
        return false;
      if(d(2)<0)
        return false;
      return true;
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_QUADRIC_H
#endif
