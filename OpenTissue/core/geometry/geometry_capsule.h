#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CAPSULE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CAPSULE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/core/geometry/geometry_compute_perpendicular_line_point.h>
#include <OpenTissue/core/function/function_signed_distance_function.h>
#include <OpenTissue/utility/utility_class_id.h>
#include <OpenTissue/core/math/math_functions.h>

#include <cmath>

namespace OpenTissue
{

  namespace geometry
  {

    /**
     * Capsule
     * - defined by 2 end points and a radius
     */
    template<typename math_types_>
    class Capsule
      : public VolumeShape< math_types_ >
      , public function::SignedDistanceFunction< math_types_ >
      , public OpenTissue::utility::ClassID< Capsule<math_types_> >
    {
    public:

      typedef          math_types_                              math_types;
      typedef typename math_types::value_traits                 value_traits;
      typedef typename math_types::real_type                    real_type;
      typedef typename math_types::vector3_type                 vector3_type;
      typedef typename math_types::matrix3x3_type               matrix3x3_type;
      typedef typename math_types::quaternion_type              quaternion_type;

    protected:

      vector3_type  m_point0;     ///< End Point 0
      vector3_type  m_point1;     ///< End Point 1
      real_type     m_radius;       ///< Radius

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Capsule<math_types_> >::class_id(); }

      Capsule() 
        : m_point0(value_traits::zero())
        , m_point1(value_traits::zero())
        , m_radius(value_traits::one())
      {}


      Capsule(vector3_type const & point0, vector3_type const & point1, real_type const & radius) 
      {
        set(point0, point1, radius);
      }

      virtual ~Capsule()
      {}

    public:

      void set(vector3_type const & point0, vector3_type const & point1, real_type const & radius)
      {
        m_point0 = point0;
        m_point1 = point1;
        m_radius = radius;
      }

      vector3_type       & point0()       { return m_point0; }
      vector3_type const & point0() const { return m_point0; }
      vector3_type       & point1()       { return m_point1; }
      vector3_type const & point1() const { return m_point1; }

      real_type       & radius()       { return m_radius; }
      real_type const & radius() const { return m_radius; }

    public:

      vector3_type get_support_point(vector3_type const & /*v*/) const { return vector3_type();  }

      real_type volume() const
      {
        real_type const h = length(m_point1-m_point0);
        real_type const tmp = value_traits::pi()*m_radius*m_radius;
        real_type const Vcylinder = tmp*h;
        real_type const Vsphere = (4./3.)*tmp*m_radius;
        return Vcylinder+Vsphere;
      }

      real_type area() const
      {
        real_type const h = length(m_point1-m_point0);
        real_type const tmp = value_traits::two()*value_traits::pi()*m_radius;
        real_type const Scylinder = tmp*h;
        real_type const Ssphere = value_traits::two()*tmp*m_radius;
        return Scylinder+Ssphere;
      }

      real_type diameter() const
      {
        // TODO: Huh? define the diameter for a capsule, please!
        //--- KE 2006-02-07: Use length of diagonal of tightest fitting OBB... 
        return value_traits::zero();
      }

      void compute_surface_points(std::vector<vector3_type>& /*points*/) const
      {
        // TODO: What corners? This function is lame!
        //--- KE 2006-02-07: It is used to get a point sampling of the surface, maybe we should re-factor the name to something more meaningfull... 
        //--- 2006-02-01 KE: Method have been renamed to a more meaningfull name.
      }

      vector3_type center() const    {      return vector3_type(  (m_point0+m_point1)/value_traits::two()   );    }

    public:

      /**
       *  Implicit Capsule.
       *
       *  p0, p1 : end points
       *  r      : radius
       *
       *  The idea is plain simple:
       *  1) Construct a line l(t) := p0+t*(p1-p0)
       *  2) Find the shortest distance from p to the line l(t)
       *     find t such that l(t)-p is perpendicular to p1-p0, e.g. (l(t)-p)*(p1-p0) = 0
       *  3) Clamp t to [0;1], and determine x = l(t)
       *     this nice trick ensures we can handle both the cylinder and the hemispheres equally
       *  4) Set F(p) := |x-p| - r
       *     F(p) < 0  <--  p inside capsule
       *     F(p) = 0  <--  p on capsule surface
       *     F(p) > 0  <--  p outside capsule
       */
      real_type evaluate(vector3_type const & x) const
      {
        vector3_type const l = closest_point_on_line_segment(x)-x;
        return l*l - m_radius*m_radius;
      }

      vector3_type gradient(vector3_type const & x) const
      {
        // TODO: differentiate a capsule :)
        return vector3_type();
      }

      vector3_type normal(vector3_type const & x) const
      {
        vector3_type const dir = x-closest_point_on_line_segment(x);
        return unit(dir);
      }

      /**
       * Returns the point on the line seqment (p1-p0) that results in the shortest distance to p
       */
      real_type signed_distance(vector3_type const & x) const
      {
        vector3_type const l = x-closest_point_on_line_segment(x);
        return length(l) - m_radius;
      }

      /**
       * Returns the point on the line seqment (p1-p0) that results in the shortest distance to p
       */
      vector3_type closest_point_on_line_segment(vector3_type const & x) const
      {
        real_type t;
        compute_perpendicular_line_point(m_point0, m_point1, x, t);
        return m_point0+math::clamp_zero_one(t)*(m_point1-m_point0);
      }

    public:

      void translate(vector3_type const & T)
      {
        m_point0 += T;
        m_point1 += T;
      }

      void rotate(matrix3x3_type const & R)
      {
        vector3_type const c = this->center();
        m_point0 = R*(m_point0-c) + c;
        m_point1 = R*(m_point1-c) + c;
      }

      void scale(real_type const & s)
      {
        vector3_type const c = this->center();

        m_point0 = s*(m_point0-c) + c;
        m_point1 = s*(m_point1-c) + c;
        m_radius *= s;
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
        using std::min;
        using std::max;

        min_coord = min(m_point0,m_point1);
        max_coord = max(m_point0,m_point1);
        min_coord -= vector3_type( m_radius, m_radius, m_radius );
        max_coord += vector3_type( m_radius, m_radius, m_radius );
      }


    };

  }  // namespace geometry

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CAPSULE_H
#endif
