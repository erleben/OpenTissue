#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SPHERE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SPHERE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_volume_shape.h>
#include <OpenTissue/core/geometry/geometry_compute_sphere_aabb.h>
#include <OpenTissue/core/function/function_signed_distance_function.h>
#include <OpenTissue/utility/utility_class_id.h>

#include <OpenTissue/core/containers/mesh/polymesh/polymesh.h>      
#include <OpenTissue/core/containers/mesh/common/util/mesh_make_sphere.h>    
#include <OpenTissue/core/containers/mesh/common/util/mesh_coordinate_iterator.h>    

#include <cassert>
#include <algorithm>

namespace OpenTissue
{

  namespace geometry
  {

    template< typename math_types_ >
    class Sphere
      : public VolumeShape< math_types_ >
      , public function::SignedDistanceFunction< math_types_ >
      , public OpenTissue::utility::ClassID< Sphere<math_types_> >
    {
    public:

      typedef          math_types_                                math_types;
      typedef typename math_types::value_traits                   value_traits;
      typedef typename math_types::real_type                      real_type;
      typedef typename math_types::vector3_type                   vector3_type;
      typedef typename math_types::matrix3x3_type                 matrix3x3_type;
      typedef typename math_types::quaternion_type                quaternion_type;

    protected:

      vector3_type m_c;  ///< The center of the sphere
      real_type    m_r;  ///< The radius
      real_type    m_r_sqr; ///< The squared radius

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::Sphere<math_types_> >::class_id(); }

      virtual ~Sphere() {}

      Sphere()
        : m_c(value_traits::zero(),value_traits::zero(),value_traits::zero())
        , m_r(value_traits::zero())
        , m_r_sqr(value_traits::zero())
      {}

      Sphere(Sphere const & s)    {      set(s);    }

      explicit Sphere(vector3_type const & center_, real_type const & radius_)    {      set(center_,radius_);    }

      real_type volume()    const    {      return (value_traits::four()*value_traits::pi()*m_r_sqr*m_r)/value_traits::three();           }
      real_type diameter()  const    {      return value_traits::two()*m_r;                       }
      real_type area()      const    {      return value_traits::four()*value_traits::pi()*m_r_sqr;                     }
      real_type perimeter() const    {      return value_traits::two()*value_traits::pi()*m_r;    }

      void set(Sphere const & sphere_)
      {
        m_c  = sphere_.m_c;
        m_r  = sphere_.m_r;
        m_r_sqr = sphere_.m_r_sqr;
      }

      void set(vector3_type const & center_,real_type const & radius_)
      {
        center(center_);
        radius(radius_);
      }

      real_type const    & radius() const { return m_r; }
      vector3_type         center() const { return m_c; }

      void radius(real_type const & value)
      {
        assert(value>=value_traits::zero() || !"Sphere::radius(value): Value must be non-negative");
        m_r = value;
        m_r_sqr = m_r*m_r;
      }

      real_type const & squared_radius() const    {      return m_r_sqr;       }
      void center(vector3_type const & value)    {      m_c = value;    }

      void compute_surface_points(std::vector<vector3_type> & points) const
      {
        typedef polymesh::PolyMesh<math_types>      mesh_type;
        typedef mesh::CoordinateIterator<mesh_type> coordinate_iterator;

        using std::fabs;

        if( fabs(m_r) > value_traits::zero())
        {
          mesh_type mesh;
          mesh::make_sphere(m_r,12,12,mesh);
          coordinate_iterator  c(mesh.vertex_begin());
          coordinate_iterator  cend(mesh.vertex_end());
          for(;c!=cend;++c)
            points.push_back( (*c + m_c));
        }
        else
          points.push_back( m_c );
      }

      void translate(vector3_type const & T)    {      m_c += T;    }
      void rotate(matrix3x3_type const & /*R*/)    {}
      void scale(real_type const & s)    {
        assert(s>=value_traits::zero() || !"Sphere::scale(s): s must be non-negative");
        m_r *= s;
        m_r_sqr = m_r*m_r;
      }
      vector3_type get_support_point(vector3_type const & v) const
      {
        return vector3_type(m_c + unit(v)*m_r);
      }

    public:

      real_type evaluate(vector3_type const & x) const
      {
        return sqr_length(x-m_c)-m_r_sqr;
      }

      vector3_type gradient(vector3_type const & x) const
      {
        return value_traits::two()*(x-m_c);
      }

      vector3_type normal(vector3_type const & x) const
      {
        return unit(gradient(x));
      }

      real_type signed_distance(vector3_type const & x) const
      {
        return length(x-m_c) - m_r;
      }

      real_type get_distance(vector3_type const & x) const
      {
        using std::fabs;
        return fabs(signed_distance(x));
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
        , matrix3x3_type const & /*R*/
        , vector3_type & min_coord
        , vector3_type & max_coord
        ) const
      {
        OpenTissue::geometry::compute_sphere_aabb(r, this->radius(), min_coord, max_coord);
      }

    };

  }  // namespace geometry

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_SPHERE_H
#endif
