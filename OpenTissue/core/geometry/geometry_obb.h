#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/core/geometry/geometry_compute_obb_aabb.h>

#include <OpenTissue/core/function/function_signed_distance_function.h>
#include <OpenTissue/utility/utility_class_id.h>
#include <OpenTissue/core/math/math_rotation.h>
#include <OpenTissue/core/math/math_precision.h>
#include <cassert>

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * A Oriented Bounding Box.
    */
    template< typename math_types_ >
    class OBB
      : public VolumeShape< math_types_ >
      , public function::SignedDistanceFunction< math_types_ >
      , public OpenTissue::utility::ClassID< OBB<math_types_> >
    {
    public:

  	  typedef          math_types_					        math_types;
      typedef typename math_types::value_traits     value_traits;
      typedef typename math_types::real_type			  real_type;
      typedef typename math_types::vector3_type		  vector3_type;
      typedef typename math_types::matrix3x3_type		matrix3x3_type;
      //typedef typename math_types::quaternion_type	quaternion_type;
      typedef typename math_types::coordsys_type		coordsys_type;

    protected:

      matrix3x3_type m_R;   ///< The orientation of the axes of this oriented bounding box.
      vector3_type   m_T;   ///< The location of this oriented bounding boxs center.
      vector3_type   m_ext; ///< The extends along this oriented bounding boxs axes.
      vector3_type   m_eps; ///< The extents and the collision envelope along this oriented bounding boxs axes.

    public:

      void compute_surface_points(std::vector<vector3_type> & points) const
      {
        vector3_type v1(m_R(0,0),m_R(1,0),m_R(2,0));
        vector3_type v2(m_R(0,1),m_R(1,1),m_R(2,1));
        vector3_type v3(m_R(0,2),m_R(1,2),m_R(2,2));

        v1 *= m_ext[0];
        v2 *= m_ext[1];
        v3 *= m_ext[2];

        vector3_type p000 = m_T - v1 - v2 - v3;
        vector3_type p001 = m_T + v1 - v2 - v3;
        vector3_type p010 = m_T - v1 + v2 - v3;
        vector3_type p011 = m_T + v1 + v2 - v3;
        vector3_type p100 = m_T - v1 - v2 + v3;
        vector3_type p101 = m_T + v1 - v2 + v3;
        vector3_type p110 = m_T - v1 + v2 + v3;
        vector3_type p111 = m_T + v1 + v2 + v3;

        points.push_back(p000);
        points.push_back(p001);
        points.push_back(p010);
        points.push_back(p011);
        points.push_back(p100);
        points.push_back(p101);
        points.push_back(p110);
        points.push_back(p111);
      }

      vector3_type           center()      const    {      return vector3_type(m_T);    }
      vector3_type   const & eps()         const    {      return m_eps;    }
      vector3_type   const & ext()         const    {      return m_ext;    }
      matrix3x3_type const & orientation() const    {      return m_R;      }

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::OBB<math_types_> >::class_id(); }

      OBB()
      {
        using OpenTissue::math::diag;

        m_T.clear();
        m_R = diag(value_traits::one());
        m_ext = vector3_type(value_traits::half(),value_traits::half(),value_traits::half());
        m_eps = vector3_type(m_ext);
        real_type epsilon = OpenTissue::math::working_precision<real_type>();
        m_eps += vector3_type(epsilon,epsilon,epsilon);
      }

      virtual ~OBB() {}

      explicit OBB(vector3_type const & T,matrix3x3_type const & R,vector3_type const & a)    {      set(T,R,a);    }

      OBB(OBB const & box)    {      set(box);    }

    public:

      /**
      * Assignment method.
      * Assigns values to this OBB, such that it is equal to the specified OBB.
      *
      * @param box
      *
      */
      void set(OBB const & box)
      {
        m_T = box.m_T;
        m_R = box.m_R;
        m_ext = box.m_ext;
        m_eps = box.m_eps;
      }

      /**
      * Assignment method.
      * Assigns values to this OBB, such that it is equal to the specified parameters.
      *
      * @param T
      * @param R
      * @param a
      */
      void set(vector3_type const & T,matrix3x3_type const & R,vector3_type const & a)
      {
        m_T = T;
        m_R = R;
        m_ext = a;
        real_type epsilon = OpenTissue::math::working_precision<real_type>();
        m_eps[0] = a[0] + epsilon;
        m_eps[1] = a[1] + epsilon;
        m_eps[2] = a[2] + epsilon;
      }

      void init(real_type const & width,real_type const & height,real_type const & depth)
      {
        m_ext = vector3_type(width,height,depth)/value_traits::two();
        m_eps = m_ext;
        real_type epsilon = OpenTissue::math::working_precision<real_type>();
        m_eps += vector3_type(epsilon,epsilon,epsilon);
      }

      void place(vector3_type const & T,matrix3x3_type const & R)
      {
        this->m_R = R;
        this->m_T = T;
      }

      void place(coordsys_type const & X)
      {
        this->m_R = X.Q();
        this->m_T = X.T();
      }

      /**
      * Apply Transform to OBB
      * First rotate then translate
      *
      * @param T
      * @param R
      */
      void xform(vector3_type const & T,matrix3x3_type const & R)
      {
        this->m_T = (R * this->m_T)  + T;
        this->m_R = R * this->m_R;
      }

      /**
      * Apply Transform to OBB
      *
      * @param X
      */
      void xform(coordsys_type const & X)
      {
        X.xform_point(m_T);
        X.xform_matrix(m_R);
      }

      /**
      * Get Diameter of OBB.
      * This is defined to be the length of the diagonal.
      *
      * @return
      */
      real_type diameter() const
      {
        using OpenTissue::math::length;

        return value_traits::two()*length(m_ext);
      }

      /**
      * Get Surface Area of OBB
      *
      * @return
      */
      real_type area() const
      {
        return value_traits::eight()*(m_ext*m_ext);
      }

      /**
      * Get Volume of OBB
      *
      * @return
      */
      real_type volume() const
      {
        return value_traits::eight()*m_ext[0]*m_ext[1]*m_ext[2];
      }

      void translate(vector3_type const & T)
      {
        m_T += T;
      }

      void rotate(matrix3x3_type const & R)
      {
        m_R = R * m_R;
      }

      void scale(real_type const & s)
      {
        assert(s>=value_traits::zero() || !"OBB::scale: scale value must be non-negative");
        m_ext *= s;
        real_type epsilon = math::working_precision<real_type>();
        m_eps[0] = m_ext[0] + epsilon;
        m_eps[1] = m_ext[1] + epsilon;
        m_eps[2] = m_ext[2] + epsilon;
      }

      vector3_type get_support_point(vector3_type const & v) const
      {
        using OpenTissue::math::unit;

        vector3_type dir,p,tmp;
        p.clear();
        dir = unit(v);

        tmp = vector3_type( m_R(0,0), m_R(1,0), m_R(2,0))* m_ext(0);
        real_type tst = tmp * dir;
        
        real_type sign = value_traits::zero();
        
        if(tst>value_traits::zero())
          sign = value_traits::one();
        if(tst<value_traits::zero())
          sign = -value_traits::one();
        // TODO: Comparing floats with == or != is not safe
        if(tst==value_traits::zero())
          sign = value_traits::zero();
        
        p += tmp*sign;

        tmp = vector3_type( m_R(0,1), m_R(1,1), m_R(2,1))* m_ext(1);
        tst = tmp * dir;
        if(tst>value_traits::zero())
          sign = value_traits::one();
        if(tst<value_traits::zero())
          sign = -value_traits::one();
        // TODO: Comparing floats with == or != is not safe
        if(tst==value_traits::zero())
          sign = value_traits::zero();
        p += tmp*sign;

        tmp = vector3_type( m_R(0,2), m_R(1,2), m_R(2,2))* m_ext(2);
        tst = tmp * dir;
        if(tst>value_traits::zero())
          sign = value_traits::one();
        if(tst<value_traits::zero())
          sign = -value_traits::one();
        // TODO: Comparing floats with == or != is not safe
        if(tst==value_traits::zero())
          sign = value_traits::zero();
        p += tmp*sign;

        p += m_T;
        return p;
      }

    public:
      
      real_type evaluate(vector3_type const & x) const 
      {
        using std::fabs;
        using OpenTissue::math::trans;
        using OpenTissue::math::max_value;

        vector3_type const p = trans(m_R)*(x-m_T);
        vector3_type const tmp = fabs(p)-m_ext;
        real_type const res = max_value(tmp);
        return res;
      }

      vector3_type gradient(vector3_type const & /*x*/) const
      {
        // TODO
        return vector3_type();
      }

      vector3_type normal(vector3_type const & /*x*/) const
      {
        // TODO
        return vector3_type();
      }

      real_type signed_distance(vector3_type const & /*x*/) const
      {
        // TODO
        return value_traits::zero();
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
          vector3_type const & r
        , matrix3x3_type const & R
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {
        OpenTissue::geometry::compute_obb_aabb(r, R, this->ext(), min_coord, max_coord);
      }


    };

  }  // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_H
#endif
