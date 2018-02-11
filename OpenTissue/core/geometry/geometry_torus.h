#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TORUS_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TORUS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/core/function/function_signed_distance_function.h>
#include <OpenTissue/utility/utility_class_id.h>

#include <cmath>  // Needed for min and max in shortestDistance


namespace OpenTissue
{

  namespace geometry
  {

// 2007-07-24 kenny: unit-test?
//   2007-07-24 micky: in due time.
//   2007-09-25 micky: There's quite a lot to unit test, how to proceed? However, I suggest UT is postposed until this class has been finalized.

    /**
     * Standard single-holed torus.
     * - In Cartesian coordinates this torus is azimuthally symmetric about the z-axis.
     */
    template<typename math_types_>
    class Torus
      : public VolumeShape< math_types_ >
      , public function::SignedDistanceFunction< math_types_ >
      , public OpenTissue::utility::ClassID< Torus<math_types_> >
    {
    public:

      typedef          math_types_                              math_types;
      typedef typename math_types::value_traits                 value_traits;
      typedef typename math_types::real_type                    real_type;
      typedef typename math_types::vector3_type                 vector3_type;
      typedef typename math_types::matrix3x3_type               matrix3x3_type;
      typedef typename math_types::quaternion_type              quaternion_type;

    protected:

      vector3_type  m_center;  ///< the center of the torus hole
      real_type     m_radius;  ///< the radius from the center to the tube center
      real_type     m_tube;    ///< the radius of the torus tube

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Torus<math_types_> >::class_id(); }

      Torus() 
        : m_center(value_traits::zero())
        , m_radius(value_traits::zero())
        , m_tube(value_traits::zero())
      {}

      /**
       * Torus constructor
       *
       * @param center  The cventer of the torus
       * @param radius  The radius from the center to the middle of the torus tube
       * @param tube    The radius of the torus tube
       *
       * @note          if radius > tube a ring torus (doughnut) is constructed.
       * @note          if radius = tube a horn torus is constructed which is tangent to itself at the center.
       * @note          if radius < tube a self-intersecting spindle torus is constructed.
       */
      Torus(vector3_type const & center, real_type const & radius, real_type const & tube)
      {
        set(center, radius, tube);
      }

      virtual ~Torus()
      {}

    public:

      /**
       * Set Torus (constructs the torus)
       *
       * @param center  The cventer of the torus
       * @param radius  The radius from the center to the middle of the torus tube
       * @param tube    The radius of the torus tube
       *
       * @note          if radius > tube a ring torus (doughnut) is constructed.
       * @note          if radius = tube a horn torus is constructed which is tangent to itself at the center.
       * @note          if radius < tube a self-intersecting spindle torus is constructed.
       */
      void set(vector3_type const & center, real_type const & radius, real_type const & tube)
      {
        m_center = center;
        m_radius = radius;
        m_tube = tube;
      }

      vector3_type      center() const { return m_center; }
      real_type const & radius() const { return m_radius; }
      real_type const & tube() const   { return m_tube; }

    public:

      vector3_type get_support_point(vector3_type const & /*v*/) const
      {
    	  // TODO: Implement this!
        return vector3_type();
      }

      real_type volume() const
      {
    	  return real_type(value_traits::two()*value_traits::pi()*value_traits::pi()*m_tube*m_tube*m_radius);
      }

      real_type area() const
      {
    	  return real_type(value_traits::four()*value_traits::pi()*value_traits::pi()*m_tube*m_radius);
      }

      real_type diameter() const
      {
        return real_type(m_radius+m_tube);
      }

      void compute_surface_points(std::vector<vector3_type>& /*points*/) const
      {
    	  // TODO: Implement this!
      }

    public:

      void translate(vector3_type const & T)
      {
        m_center += T;
      }

      void rotate(matrix3x3_type const & /* R */)
      {
    	  // TODO: Implement this!
      }

      void scale(real_type const & s)
      {
    	  m_radius *= s;
    	  m_tube *= s;
      }

    public:

      /**
       * implicit function of a torus:
       * f(x,y,z) = (c-sqrt(x^2+y^2))^2+z^2-a^2
       */
      real_type evaluate(vector3_type const & x) const
      {
    	  // TODO: missing arbitrary rotation
        using std::sqrt;
        vector3_type const r = x-m_center;
        real_type const tmp = m_radius-sqrt(r[0]*r[0]+r[1]*r[1]);
        return real_type(tmp*tmp+r[2]*r[2]-m_tube*m_tube);
      }

      vector3_type gradient(vector3_type const & x) const
      {
    	  // TODO: missing arbitrary rotation
        using std::sqrt;
        vector3_type const r = x-m_center;
        real_type const len = sqrt(r[0]*r[0]+r[1]*r[1]);
        if (len > value_traits::zero()) {
          real_type const tmp = -value_traits::two()*m_radius/len;
          return vector3_type(r[0]*tmp+value_traits::two()*r[0], r[1]*tmp+value_traits::two()*r[1], value_traits::two()*r[2]);
        }
        return vector3_type(value_traits::zero());
      }

      vector3_type normal(vector3_type const & x) const
      {
        // TODO: verify this!!!
// 2007-07-24 kenny: Maybe write unit( this->gradient(x) )? I think this-> is mandatory inside template classes...
//   2007-07-24 micky: Can you try to compile it on gcc? I only thought this-> was required to reference functions from derived classes.
//     2007-07-27 kenny: My linux box is broken:-( 
//       2007-09-25 micky: Can you give it a new shot?
//         2007-09-26 kenny: Still don't got any linux:-( Maybe Sune or Jesper can help out, they are linux users.

        return unit(gradient(x));
      }

      real_type signed_distance(vector3_type const & x) const
      {
        // TODO: this is so hacky, implement a true signed_distance function!
        using std::sqrt;
        real_type const f = evaluate(x);
        real_type const sign = OpenTissue::math::sgn(f);
        return real_type(boost::numeric_cast<real_type>(0.1)*sign*sqrt(sign*f));
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

}  // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_TORUS_H
#endif
