#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PRISM_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PRISM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/utility/utility_class_id.h>
#include <OpenTissue/core/math/math_constants.h>


#include <cassert>
#include <cmath>
#include <iostream>

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * A Prism.
    */
    template< typename math_types_ >
    class Prism
      : public VolumeShape< math_types_ >
      , public OpenTissue::utility::ClassID< Prism<math_types_> >
    {
    public:

      typedef          math_types_                   math_types;
      typedef typename math_types::real_type         real_type;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::matrix3x3_type    matrix3x3_type;
      typedef typename math_types::quaternion_type   quaternion_type;

    protected:

      vector3_type  m_p0;
      vector3_type  m_p1;
      vector3_type  m_p2;
      real_type     m_height;

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Prism<math_types_> >::class_id(); }

      virtual ~Prism() {}

      void compute_surface_points(std::vector<vector3_type> & points) const
      {
        vector3_type u = (m_p1-m_p0);
        vector3_type v = (m_p2-m_p0);
        vector3_type n = unit(u % v);
        points.push_back(m_p0);
        points.push_back(m_p1);
        points.push_back(m_p2);
        vector3_type p0h = m_p0 + n*m_height;
        vector3_type p1h = m_p1 + n*m_height;
        vector3_type p2h = m_p2 + n*m_height;
        points.push_back(p0h);
        points.push_back(p1h);
        points.push_back(p2h);
      }

      vector3_type center() const
      {
        vector3_type u   = (m_p1-m_p0);
        vector3_type v   = (m_p2-m_p0);
        vector3_type n   = unit( u % v);
        vector3_type tmp = ((m_p0 + m_p1 + m_p2)/3.) + (0.5*m_height)*n;
        return tmp;
      }

    public:

      real_type    const & height() const    {      return m_height;  }
      vector3_type const & p0()     const    {      return m_p0;      }
      vector3_type const & p1()     const    {      return m_p1;      }
      vector3_type const & p2()     const    {      return m_p2;      }

      Prism()
        : m_p0(-1,0,0)
        , m_p1(1,0,0)
        , m_p2(0,1,0)
        ,m_height(1.)
      {  }

      /**
      * Specialized constructor.
      *
      * @param p0
      * @param p1
      * @param p2
      * @param height
      */
      Prism(vector3_type const & p0,vector3_type const & p1,vector3_type const & p2, real_type const & height)
      {
        set(p0,p1,p2,height);
      }

      Prism(Prism const & p)  {    set(p);  }

      /**
      * Assignment method.
      *
      * @param prism
      *
      */
      void set(Prism const & p)  {  set(p.m_p0,p.m_p1,p.m_p2,p.m_height);  }

      /**
      * Assignment method.
      *
      * @param p0
      * @param p1
      * @param p2
      * @param height
      */
      void set( vector3_type const & p0_, vector3_type const & p1_, vector3_type const & p2_, real_type const & height_)
      {
        assert(height_>=0 || !"Prism::set(): Height must be non-negative");
        m_p0 = p0_;
        m_p1 = p1_;
        m_p2 = p2_;
        m_height = height_;
      }

      /**
      * Apply Transform to Prism
      * First rotate then translate
      *
      * @param T
      * @param R
      */
      void xform(vector3_type const & T, matrix3x3_type const & R)
      {
        vector3_type c = center();
        m_p0 = (R * (m_p0 - c)) + c + T;
        m_p1 = (R * (m_p1 - c)) + c + T;
        m_p2 = (R * (m_p2 - c)) + c + T;
      }

      /**
      * Get Diameter
      *
      * @return
      */
      real_type diameter() const
      {
        using std::max;

        vector3_type tmp;
        tmp = (m_p1-m_p0);
        real_type l0 = length(tmp);
        tmp = (m_p2-m_p1);
        real_type l1 = length(tmp);
        tmp = (m_p2-m_p0);
        real_type l2 = length(tmp);
        return max(l0, max(l1,l2) );
      }

      /**
      * Get Surface Area
      *
      * @return
      */
      real_type area() const
      {
        vector3_type u = (m_p1-m_p0);
        vector3_type v = (m_p2-m_p0);
        vector3_type uXv = u % v;

        vector3_type w;
        w = (m_p2-m_p1);
        return length(uXv) + (length(u) + length(v) + length(w))*m_height;
      }

      /**
      * Get Volume
      *
      * @return
      */
      real_type volume() const
      {
        vector3_type u = (m_p1-m_p0);
        vector3_type v = (m_p2-m_p0);
        vector3_type uXv = u % v;
        return 0.5*m_height*length(uXv);
      }

      /**
      *
      * @param T
      */
      void translate(vector3_type const & T)
      {
        m_p0 += T;
        m_p1 += T;
        m_p2 += T;
      }

      /**
      *
      * @param R
      */
      void rotate(matrix3x3_type const & R)
      {
        vector3_type c = center();
        m_p0 = (R*(m_p0 - c))+c;
        m_p1 = (R*(m_p1 - c))+c;
        m_p2 = (R*(m_p2 - c))+c;
      }

      /**
      *
      * @param s
      */
      void scale(real_type const & s)
      {
        assert(s>=0);
        vector3_type c = center();
        m_p0 = (s*(m_p0 - c))+c;
        m_p1 = (s*(m_p1 - c))+c;
        m_p2 = (s*(m_p2 - c))+c;
        m_height *= s;
      }

      vector3_type get_support_point(vector3_type const & v) const
      {
        vector3_type p;
        std::vector<vector3_type> c;
        compute_surface_points(c);
        real_type tst = math::detail::lowest<real_type>();
        for(int i=0;i<6;++i)
        {
          real_type tmp = v*c[i];
          if (tmp>tst)
          {
            tst = tmp;
            p = c[i];
          }
        }
        return p;
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
        assert(false || !"Sorry not implemented yet!");
      }

    };

  }  // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PRISM_H
#endif
