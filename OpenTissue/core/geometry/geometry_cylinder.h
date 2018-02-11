#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_H
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

#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h> 
#include <OpenTissue/core/containers/mesh/common/util/mesh_make_cylinder.h> 

#include <cassert>
#include <cmath>  // Needed for acos in compute_surface_points

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * An Oriented Cylinder.
    */
    template< typename math_types_ >
    class Cylinder
      : public VolumeShape< math_types_ >
      , public function::SignedDistanceFunction< math_types_ >
      , public OpenTissue::utility::ClassID< Cylinder<math_types_> >
    {
    public:

      typedef          math_types_                   math_types;
      typedef typename math_types::value_traits      value_traits;
      typedef typename math_types::real_type         real_type;
      typedef typename math_types::vector3_type      vector3_type;
      typedef typename math_types::matrix3x3_type    matrix3x3_type;
      typedef typename math_types::quaternion_type   quaternion_type;

    protected:

      vector3_type     m_axis;
      vector3_type     m_center;
      real_type        m_radius;
      real_type        m_height;

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Cylinder<math_types_> >::class_id(); }

      virtual ~Cylinder(){}

    public:

      vector3_type         center() const    {  return vector3_type(m_center);  }
      real_type    const & radius() const    {  return m_radius;                }
      real_type    const & height() const    {  return m_height;                }
      vector3_type const & axis()   const    {  return m_axis;                  }

      Cylinder()
        : m_axis(value_traits::zero(),value_traits::zero(),value_traits::one())
        , m_center(value_traits::zero(),value_traits::zero(),value_traits::zero())
        , m_radius(value_traits::one())
        , m_height(value_traits::one())
      {}

      explicit Cylinder(
        vector3_type const & new_center
        , vector3_type const & new_axis
        , real_type const & new_height
        , real_type const & new_radius
        )
      {
        set(new_center,new_axis,new_height,new_radius);
      }

      Cylinder(Cylinder const & copy)
      {
        set(copy);
      }

    public:

      void set (Cylinder const& other_cylinder)
      {
        m_center = other_cylinder.m_center;
        m_axis   = other_cylinder.m_axis;
        m_height = other_cylinder.m_height;
        m_radius = other_cylinder.m_radius;
      }

      void set (
        vector3_type const & new_center
        , vector3_type const & new_axis
        , real_type const & new_height
        , real_type const & new_radius
        )
      {
        assert(new_radius>=value_traits::zero() || !"Cylinder::set(): radius must be non-negative");
        assert(new_height>=value_traits::zero() || !"Cylinder::set(): height must be non-negative");

        this->m_center = new_center;
        this->m_axis = unit(new_axis);
        this->m_height = new_height;
        this->m_radius = new_radius;
      }

      void place(vector3_type const & new_center,vector3_type const & new_axis)
      {
        this->m_axis = unit(new_axis);
        this->m_center = new_center;
      }

      /**
      * Apply Transform to Cylinder
      * First rotate then translate
      *
      * @param T
      * @param R
      */
      void xform(vector3_type const & T,matrix3x3_type const & R)
      {
        this->m_center = R * this->m_center + T;
        this->m_axis   = R * this->m_axis;
      }

      /**
      * Get Diameter
      *
      * @return
      */
      real_type diameter() const    {      return value_traits::two()*m_radius;    }

      /**
      * Get Surface Area
      *
      * @return
      */
      real_type area() const
      {
        return value_traits::two()*value_traits::pi()*m_radius*m_height + (value_traits::two()*value_traits::pi()*m_radius*m_radius);
      }

      /**
      * Get Volume
      *
      * @return
      */
      real_type volume() const    {      return value_traits::pi()*m_radius*m_radius*m_height;    }

      void compute_surface_points(std::vector<vector3_type> & points) const
      {
        using std::acos;

        real_type rad = value_traits::zero();
        vector3_type u(value_traits::one(),value_traits::zero(),value_traits::zero());

        //--- Compute orientation of normal
        if((m_axis[0]==value_traits::zero()) && (m_axis[1]==value_traits::zero()))
        {
          if(m_axis[2]>value_traits::zero())
          {
            rad = value_traits::zero();
          }
          else
          {
            rad = value_traits::pi();
          }
        }
        else
        {
          vector3_type k(value_traits::zero(),value_traits::zero(),value_traits::one());

          u = unit(m_axis % k);
          rad = -acos(m_axis*k);
        }
        matrix3x3_type R;
        R = Ru(rad,u);

        polymesh::PolyMesh<math_types> mesh;
        mesh::make_cylinder(m_radius,m_height,12,mesh);
        mesh::CoordinateIterator<polymesh::PolyMesh<math_types> >  c(mesh.vertex_begin());
        mesh::CoordinateIterator<polymesh::PolyMesh<math_types> >  cend(mesh.vertex_end());
        for(;c!=cend;++c)
          points.push_back( (R*(*c) + m_center));
      }

      void translate(vector3_type const & T)    {      m_center += T;    }

      void rotate(matrix3x3_type const & R)    {      m_axis = R * m_axis;    }

      void scale(real_type const & s)    {      m_radius *= s;      m_height *= s;    }

      vector3_type get_support_point(vector3_type const & v) const
      {
        vector3_type c,p;
        c = unit(v);
        real_type proj = c * m_axis;
        p = m_axis;
        if(proj>value_traits::zero())
          p *= m_height;
        else if(proj<value_traits::zero())
          p *= -m_height;
        else
          p.clear();
        p += m_center;
        vector3_type a   = proj*m_axis;
        vector3_type b = (c - a);
        real_type delta = sqrt(b*b);
        // TODO: Comparing floats with == or != is not safe
        if(delta)
        {
          b *= m_radius/delta;
          p += b;
        }
        return p;
      }

      real_type evaluate(vector3_type const & x) const
      {
        // TODO: missing axis rotation
        vector3_type const r = x-m_center;
        return real_type(value_traits::one()/(m_radius*m_radius)*(r[0]*r[0]+r[1]*r[1])-m_height*m_height);
      }

      vector3_type gradient(vector3_type const & x) const
      {
        // TODO: missing axis rotation
        vector3_type const r = x-m_center;
        return vector3_type(r[0], r[1], value_traits::zero())*(value_traits::two()/(m_radius*m_radius));
      }

      vector3_type normal(vector3_type const & x) const
      {
        // TODO: verify this!!!
        return unit(gradient(x));
      }

      real_type signed_distance(vector3_type const & x) const
      {
        // TODO: verify this!!!
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
        vector3_type const & /*r*/
        , matrix3x3_type const & /*R*/
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {
        vector3_type p;

        p = get_support_point( vector3_type( value_traits::one(), value_traits::zero(), value_traits::zero() ) );
        min_coord = p;
        max_coord = p;

        p = get_support_point( vector3_type( -value_traits::one(), value_traits::zero(), value_traits::zero() ) );
        min_coord = min(min_coord, p);
        max_coord = max(min_coord, p);

        p = get_support_point( vector3_type( value_traits::zero(), value_traits::one(), value_traits::zero() ) );
        min_coord = min(min_coord, p);
        max_coord = max(min_coord, p);

        p = get_support_point( vector3_type( value_traits::zero(), -value_traits::one(), value_traits::zero() ) );
        min_coord = min(min_coord, p);
        max_coord = max(min_coord, p);

        p = get_support_point( vector3_type( value_traits::zero(), value_traits::zero(), value_traits::one() ) );
        min_coord = min(min_coord, p);
        max_coord = max(min_coord, p);

        p = get_support_point( vector3_type( value_traits::zero(), value_traits::zero(), -value_traits::one() ) );
        min_coord = min(min_coord, p);
        max_coord = max(min_coord, p);
      }

    };

  }  // namespace geometry

} // namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_H
#endif
