#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ELLIPSOID_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ELLIPSOID_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_eigen_system_decomposition.h>
#include <OpenTissue/core/math/math_constants.h>
#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/core/geometry/geometry_compute_obb_aabb.h>
#include <OpenTissue/utility/utility_class_id.h>

#include <cmath>

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * Ellipsoid.
    */
    template< typename math_types_ >
    class Ellipsoid
      : public VolumeShape< math_types_ >
      , public OpenTissue::utility::ClassID< Ellipsoid<math_types_> >
    {
    public:

      typedef          math_types_                            math_types;
      typedef typename math_types::index_type                 index_type;
      typedef typename math_types::value_traits               value_traits;
      typedef typename math_types::real_type                  real_type;
      typedef typename math_types::vector3_type               vector3_type;
      typedef typename math_types::matrix3x3_type             matrix3x3_type;
      typedef typename math_types::quaternion_type            quaternion_type;
      typedef          Ellipsoid<math_types_>                 ellipsoid_type;

    protected:

      vector3_type m_center;    ///< The center of the ellipsoid.
      vector3_type m_axis0;     ///< First axis.
      vector3_type m_axis1;     ///< Second axis.
      vector3_type m_axis2;     ///< Third axis.
      real_type    m_radius0;   ///< Radius of first axis.
      real_type    m_radius1;   ///< Radius of first axis.
      real_type    m_radius2;   ///< Radius of first axis.

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Ellipsoid<math_types_> >::class_id(); }

      Ellipsoid()
        : m_center(value_traits::zero(),value_traits::zero(),value_traits::zero())
        , m_axis0(value_traits::one(),value_traits::zero(),value_traits::zero())
        , m_axis1(value_traits::zero(),value_traits::one(),value_traits::zero())
        , m_axis2(value_traits::zero(),value_traits::zero(),value_traits::one())
        , m_radius0(value_traits::one())
        , m_radius1(value_traits::one())
        , m_radius2(value_traits::one())
      {}

      virtual ~Ellipsoid() {}

    public:

      Ellipsoid & operator=(ellipsoid_type const & ellipsoid)
      {
        m_center  = ellipsoid.m_center;
        m_axis0   = ellipsoid.m_axis0;
        m_axis1   = ellipsoid.m_axis1;
        m_axis2   = ellipsoid.m_axis2;
        m_radius0 = ellipsoid.m_radius0;
        m_radius1 = ellipsoid.m_radius1;
        m_radius2 = ellipsoid.m_radius2;
        return *this;
      }

    public:

      real_type          & radius0()       { return m_radius0; }
      real_type          & radius1()       { return m_radius1; }
      real_type          & radius2()       { return m_radius2; }
      real_type const    & radius0() const { return m_radius0; }
      real_type const    & radius1() const { return m_radius1; }
      real_type const    & radius2() const { return m_radius2; }
      vector3_type       & center()        { return m_center;  }
      vector3_type /*const &*/ center()  const { return m_center;  } //--- To comply with VolumeAdapter interface...
      vector3_type       & axis0()         { return m_axis0;   }
      vector3_type       & axis1()         { return m_axis1;   }
      vector3_type       & axis2()         { return m_axis2;   }
      vector3_type const & axis0()   const { return m_axis0;   }
      vector3_type const & axis1()   const { return m_axis1;   }
      vector3_type const & axis2()   const { return m_axis2;   }


      /**
      * Get Radius.
      *
      * @param idx  Index of the axis along which the radius is wanted.
      *
      * @return     The radius of the specified axis.
      */
      real_type & radius(index_type idx)
      {
        assert(idx>=0 || !"Ellipsoid::radius(): Index out of bounds");
        assert(idx<3  || !"Ellipsoid::radius(): Index out of bounds");
        if(idx==0)
          return m_radius0;
        else if(idx==1)
          return m_radius1;
        return m_radius2;
      }


      /**
      * Get Radius.
      *
      * @param idx  Index of the axis along which the radius is wanted.
      *
      * @return     The radius of the specified axis.
      */
      real_type const & radius(index_type idx) const
      {
        assert(idx>=0 || !"Ellipsoid::radius(): Index out of bounds");
        assert(idx<3  || !"Ellipsoid::radius(): Index out of bounds");
        if(idx==0)
          return m_radius0;
        else if(idx==1)
          return m_radius1;
        return m_radius2;
      }


      /**
      * Get Axis.
      *
      * @param idx  Index of the axis.
      *
      * @return     The specified axis.
      */
      vector3_type const & axis(index_type idx) const
      {
        assert(idx>=0 || !"Ellipsoid::axis(): Index out of bounds");
        assert(idx<3  || !"Ellipsoid::axis(): Index out of bounds");
        if(idx==0)
          return m_axis0;
        else if(idx==1)
          return m_axis1;
        return m_axis2;
      }

      /**
      * Get Axis.
      *
      * @param idx  Index of the axis.
      *
      * @return     The specified axis.
      */
      vector3_type & axis(index_type idx)
      {
        assert(idx>=0 || !"Ellipsoid::axis(): Index out of bounds");
        assert(idx<3  || !"Ellipsoid::axis(): Index out of bounds");
        if(idx==0)
          return m_axis0;
        else if(idx==1)
          return m_axis1;
        return m_axis2;
      }

      void orientation(matrix3x3_type const & R)
      {
        m_axis0(0) = R(0,0);
        m_axis0(1) = R(1,0);
        m_axis0(2) = R(2,0);

        m_axis1(0) = R(0,1);
        m_axis1(1) = R(1,1);
        m_axis1(2) = R(2,1);

        m_axis2(0) = R(0,2);
        m_axis2(1) = R(1,2);
        m_axis2(2) = R(2,2);
      }

      matrix3x3_type orientation() const
      {
        return matrix3x3_type(
          m_axis0(0), m_axis1(0), m_axis2(0)
          , m_axis0(1), m_axis1(1), m_axis2(1)
          , m_axis0(2), m_axis1(2), m_axis2(2)
          );
      }

    public:

      real_type volume() const { return (4.0/3.0)*value_traits::pi()*m_radius0*m_radius1*m_radius2; }

    public:

      /**
      *  Ellipsoid (Implicit) Equation.
      *
      *  let p = [x y z]^T, then an ellipsoid can be written as (in homegenious coordinates)
      *
      *             | A   B |  | p |
      *   |p^T 1 |  | B^T C |  | 1 | =  p^T A p + 2 B^T p + C = 0
      *
      * Which should be equivalent to the classical ellipoid equation
      *
      *   (R^T (p - center) )_x^2 / r_0^2 + ...+ (R^T (p - center) )_z^2 / r_2^2 = 1
      *
      * With some mathemagical handy-work is can be shown that
      *
      *  D = diag( 1/r0^2, 1/r1^2, 1/r2^2)
      *  R = (axis0, axis1, axis2)
      *  A = R D R^T   // Note this is a symmetric matrix
      *  B = - A center
      *  C = center^T A center - 1  =  - center^T B - 1
      *
      */
      void get_equation(matrix3x3_type & A,vector3_type & B,real_type & C)const
      {
        assert(m_radius0>value_traits::zero() || !"Ellipsoid::get_equation(): First radius was zero");
        assert(m_radius1>value_traits::zero() || !"Ellipsoid::get_equation(): Second radius was zero");
        assert(m_radius2>value_traits::zero() || !"Ellipsoid::get_equation(): Third radius was zero");

        //--- Compute D = diag(1/r0^21/r1^2,1/r2^2)
        real_type d0 = value_traits::one()/(m_radius0*m_radius0);
        real_type d1 = value_traits::one()/(m_radius1*m_radius1);
        real_type d2 = value_traits::one()/(m_radius2*m_radius2);
        //--- Compute A = R D R^T
        for(unsigned int i=0;i<3;++i)
          for(unsigned int j=0;j<3;++j)
            A(i,j) = m_axis0(i)*d0*m_axis0(j) + m_axis1(i)*d1*m_axis1(j) + m_axis2(i)*d2*m_axis2(j);
        B = - A*m_center;
        C = - m_center * B - value_traits::one();
      }

      /**
      * Set Ellipsoid
      *
      * Let ellipsoid equation be given as
      *
      *    p^T A p + 2 B^T p + C = 0
      *
      * This method extracts, ellipsoid axes and center from this implicit equation.
      */
      void set_equation(matrix3x3_type const & A,vector3_type const & B,real_type const & C)
      {
        using std::sqrt;
        using std::fabs;

        //--- An ellipsoid is defined as
        //---
        //---   (x-m)^T H (x-m) = 1
        //---
        //--- We have
        //---
        //---   x^T A x + 2 B^T x + C = 0
        //---
        //--- At the center of the ellipsoid the gradient vanishes, this leads to
        //---
        //---   A m = -B  or B = - A m
        //---
        //--- Substitution gives
        //---
        //---   x^T A x - m^T A x - x^T A m + C = 0
        //---
        //--- Adding zero to the equation gives
        //---
        //---   x^T A x - m^T A x - x^T A m +  m^T A m  - m^T A m + C = 0
        //---
        //--- Which is
        //---
        //---   (x-m)^T A (x - m)  - m^T A m + C = 0
        //---   (x-m)^T A (x - m)                =  m^T A m - C
        //---
        //--- From which it is clear that
        //---
        //---    H =  A / ( - m^T B - C)
        //---
        //--- H contains information about axes and radius, since H = R^T D R
        //---
        matrix3x3_type M = inverse(A);
        m_center = -M*B;  //--- Gradient vanished at center,  d/dx ( x^T A x + 2 B^T x + C) =  2A x + 2B = 0 => x = - inv(A) B
        real_type normalization = -C - m_center*B;
        M = A;
        M /= normalization;
        matrix3x3_type V;
        vector3_type d;
        math::eigen(M,V,d);
        //--- Eigenvalues are required to be positive otherwise quadric is not an ellipsoid!!!!
        //assert(d(0)>0 || !"Ellipsoid::set_equation(): Non-positive eigenvalue encountered");
        //assert(d(1)>0 || !"Ellipsoid::set_equation(): Non-positive eigenvalue encountered");
        //assert(d(2)>0 || !"Ellipsoid::set_equation(): Non-positive eigenvalue encountered");
        d(0) = d(0) < value_traits::zero() ? - d(0) : d(0);
        d(1) = d(1) < value_traits::zero() ? - d(1) : d(1);
        d(2) = d(2) < value_traits::zero() ? - d(2) : d(2);

        m_radius0 = sqrt( value_traits::one()/fabs(d(0)) );
        m_radius1 = sqrt( value_traits::one()/fabs(d(1)) );
        m_radius2 = sqrt( value_traits::one()/fabs(d(2)) );
        m_axis0(0) = V(0,0);      m_axis0(1) = V(1,0);      m_axis0(2) = V(2,0);
        m_axis1(0) = V(0,1);      m_axis1(1) = V(1,1);      m_axis1(2) = V(2,1);
        m_axis2(0) = V(0,2);      m_axis2(1) = V(1,2);      m_axis2(2) = V(2,2);
      }

    public:

      /**
      * Retrive Covariance Information about Ellipsoid.
      *
      * @param C      Covariance matrix.
      * @param mean   Mean point.
      */
      void get_covariance(vector3_type & mean, matrix3x3_type & C)
      {
        mean = center;
        assert(m_radius0>value_traits::zero() || !"Ellipsoid::get_covariance(): radius of first axis was zero");
        assert(m_radius1>value_traits::zero() || !"Ellipsoid::get_covariance(): radius of second axis was zero");
        assert(m_radius2>value_traits::zero() || !"Ellipsoid::get_covariance(): radius of third axis was zero");
        //--- Compute D = diag(1/r0^21/r1^2,1/r2^2)
        real_type d0 = value_traits::one()/(m_radius0*m_radius0);
        real_type d1 = value_traits::one()/(m_radius1*m_radius1);
        real_type d2 = value_traits::one()/(m_radius2*m_radius2);
        //--- Compute C = R D R^T
        for(unsigned int i=0;i<3;++i)
          for(unsigned int j=0;j<3;++j)
            C(i,j) = m_axis0(i)*d0*m_axis0(j) + m_axis1(i)*d1*m_axis1(j) + m_axis2(i)*d2*m_axis2(j);
      }

      /**
      * Set Ellipsoid to match Covariance Matrix.
      *
      * @param C      Covariance matrix.
      * @param mean   Mean point.
      */
      void set_covariance(vector3_type const & mean,matrix3x3_type const & C)
      {
        using std::sqrt;
        using std::fabs;

        matrix3x3_type M,V;
        vector3_type d;
        M = C;
        math::eigen(M,V,d);
        assert(d(0)>value_traits::zero() || !"Ellipsoid::set_covariance(): first diagonal entry was zero");
        assert(d(1)>value_traits::zero() || !"Ellipsoid::set_covariance(): second diagonal entry was zero");
        assert(d(2)>value_traits::zero() || !"Ellipsoid::set_covariance(): third diagonal entry was zero");

        m_radius0 = sqrt(value_traits::one()/fabs(d(0)));
        m_radius1 = sqrt(value_traits::one()/fabs(d(1)));
        m_radius2 = sqrt(value_traits::one()/fabs(d(2)));
        m_axis0(0) = V(0,0);      m_axis0(1) = V(1,0);      m_axis0(2) = V(2,0);
        m_axis1(0) = V(0,1);      m_axis1(1) = V(1,1);      m_axis1(2) = V(2,1);
        m_axis2(0) = V(0,2);      m_axis2(1) = V(1,2);      m_axis2(2) = V(2,2);
        m_center = mean;
      }

    public:

      /**
      * Set Ellipsoid to be Spherical.
      *
      * @param radius   The radius of the sphere.
      * @param center   The cetner of the sphere.
      */
      void set_sphere(real_type const & radius,vector3_type const & center)
      {
        m_radius0 = m_radius1 = m_radius2 = radius;
        m_center = center;
        m_axis0 = vector3_type(value_traits::one(),value_traits::zero(),value_traits::zero());
        m_axis1 = vector3_type(value_traits::zero(),value_traits::one(),value_traits::zero());
        m_axis2 = vector3_type(value_traits::zero(),value_traits::zero(),value_traits::one());
      }

    public:

      /**
      * Generate Random Surface Points on Ellipsoid.
      *
      * @param P   Upon return this container will hold the generated points.
      */
      template<typename point_container>
      void random_points(point_container & P)
      {
        unsigned int N = 2000;
        P.resize(N);
        for(unsigned int i=0;i<N;++i)
        {
          vector3_type p,pp;
          do
          {
            random(p,-value_traits::one(),value_traits::one());
          }
          while(p(0)==value_traits::zero() && p(1)==value_traits::zero() && p(2)==value_traits::zero());

          p = unit(p);
          p(0) *= m_radius0;
          p(1) *= m_radius1;
          p(2) *= m_radius2;
          pp(0) = m_axis0(0) * p(0) + m_axis1(0) * p(1) + m_axis2(0) * p(2) + m_center(0);
          pp(1) = m_axis0(1) * p(0) + m_axis1(1) * p(1) + m_axis2(1) * p(2) + m_center(1);
          pp(2) = m_axis0(2) * p(0) + m_axis1(2) * p(1) + m_axis2(2) * p(2) + m_center(2);
          P[i] = pp;
        }
      }

    public:

      void translate(vector3_type const & T) {      m_center = m_center + T;    }

      void rotate(matrix3x3_type const & R)
      {
        vector3_type tmp;

        tmp = m_axis0;
        m_axis0 = R*tmp;

        tmp = m_axis1;
        m_axis1 = R*tmp;

        tmp = m_axis2;
        m_axis2 = R*tmp;

        tmp = m_center;
        m_center = R*tmp;
      }

      void scale(real_type const & s)
      {
        m_radius0 *= s;
        m_radius1 *= s;
        m_radius2 *= s;
        m_center  *= s;
      }


      vector3_type get_support_point(vector3_type const & /*v*/) const   {  assert(false ||!"Ellipsoid::get_support_point(): Sorry not implemented!");    return vector3_type();    }

      real_type    area()                            const {  assert(false ||!"Ellipsoid::area(): Sorry not implemented!");    return real_type();   }
      real_type    diameter()                        const {  assert(false ||!"Ellipsoid::diameter(): Sorry not implemented!");    return real_type();   }

      void compute_surface_points(std::vector<vector3_type> & /*points*/) const {  assert(false ||!"Ellipsoid::compute_surface_points): Sorry not implemented!");   }


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
        vector3_type const & r
        , matrix3x3_type const & R
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {

        vector3_type ext = vector3_type(this->radius0(), this->radius1(), this->radius2() );
        OpenTissue::geometry::compute_obb_aabb(
          this->center()
          , this->orientation()
          , ext
          , min_coord
          , max_coord
          );

      }

    };

  }  // namespace geometry

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ELLIPSOID_H
#endif
