#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_HYBRID_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_HYBRID_H
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
#include <OpenTissue/core/geometry/geometry_aabb.h>
#include <OpenTissue/core/geometry/geometry_obb.h>
#include <OpenTissue/core/geometry/geometry_sphere.h>
#include <OpenTissue/core/geometry/geometry_cylinder.h>
#include <OpenTissue/core/geometry/geometry_prism.h>

namespace OpenTissue
{

  namespace geometry
  {

    /**
    * Hybrid Volume Class.
    */
    template< typename math_types_ >
    class HybridVolume
      : public VolumeShape< math_types_ >
      , public OpenTissue::utility::ClassID< HybridVolume<math_types_> >
    {
    public:

      typedef          math_types_                     math_types;
      typedef typename math_types::value_traits        value_traits;
      typedef typename math_types::real_type           real_type;
      typedef typename math_types::vector3_type        vector3_type;
      typedef typename math_types::matrix3x3_type      matrix3x3_type;
      typedef typename math_types::quaternion_type     quaternion_type;
      typedef VolumeShape< math_types_ >               volume_type;

      typedef AABB<math_types>                         aabb_type;
      typedef OBB<math_types>                          obb_type;
      typedef Cylinder<math_types>                     cylinder_type;
      typedef Sphere<math_types>                       sphere_type;
      typedef Prism<math_types>                        prism_type;

      typedef enum{
        selection_undefined
        , selection_sphere
        , selection_aabb
        , selection_obb
        , selection_cylinder
        , selection_prism
        , selection_tetrahedron
      } selection_type;

    public:

      aabb_type      m_aabb;
      obb_type       m_obb;
      cylinder_type  m_cylinder;
      sphere_type    m_sphere;
      prism_type     m_prism;
      selection_type m_picked;

    public:

      size_t const class_id() const { return OpenTissue::utility::ClassID<OpenTissue::geometry::HybridVolume<math_types_> >::class_id(); }

      virtual ~HybridVolume() {}

      HybridVolume()
        : m_picked(selection_undefined)
      {}

      void set(aabb_type const & other_aabb)
      {
        m_aabb = other_aabb;
        m_picked = selection_aabb;
      }

      void set(obb_type const & other_obb)
      {
        m_obb = other_obb;
        m_picked = selection_obb;
      }

      void set(sphere_type const & other_sphere)
      {
        m_sphere = other_sphere;
        m_picked = selection_sphere;
      }

      void set(cylinder_type const & other_cylinder)
      {
        m_cylinder = other_cylinder;
        m_picked = selection_cylinder;
      }

      void set(prism_type const & other_prism)
      {
        m_prism = other_prism;
        m_picked = selection_prism;
      }

      real_type volume() const
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.volume();
          break;
        case selection_obb:
          return m_obb.volume();
          break;
        case selection_sphere:
          return m_sphere.volume();
          break;
        case selection_cylinder:
          return m_cylinder.volume();
          break;
        case selection_prism:
          return m_prism.volume();
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::volume(): case not handled");
          break;
        };
        return value_traits::zero();
      }

      real_type area() const
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.area();
          break;
        case selection_obb:
          return m_obb.area();
          break;
        case selection_sphere:
          return m_sphere.area();
          break;
        case selection_cylinder:
          return m_cylinder.area();
          break;
        case selection_prism:
          return m_prism.area();
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::area(): case not handled");
          break;
        };
        return value_traits::zero();
      }

      real_type diameter() const
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.diameter();
          break;
        case selection_obb:
          return m_obb.diameter();
          break;
        case selection_sphere:
          return m_sphere.diameter();
          break;
        case selection_cylinder:
          return m_cylinder.diameter();
          break;
        case selection_prism:
          return m_prism.diameter();
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::diameter(): case not handled");
          break;
        };
        return value_traits::zero();
      }

      vector3_type center() const
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.center();
          break;
        case selection_obb:
          return m_obb.center();
          break;
        case selection_sphere:
          return m_sphere.center();
          break;
        case selection_cylinder:
          return m_cylinder.center();
          break;
        case selection_prism:
          return m_prism.center();
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::center(): case not handled");
          break;
        };
        return vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
      }


      selection_type selected_type() const
      {
        return m_picked;
      }

      void compute_surface_points(std::vector<vector3_type> & points) const
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.compute_surface_points(points);
          break;
        case selection_obb:
          return m_obb.compute_surface_points(points);
          break;
        case selection_sphere:
          return m_sphere.compute_surface_points(points);
          break;
        case selection_cylinder:
          return m_cylinder.compute_surface_points(points);
          break;
        case selection_prism:
          return m_prism.compute_surface_points(points);
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::compute_surface_points(): case not handled");
          break;
        };
      }

    public:

      void translate(vector3_type const & T)
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.translate(T);
          break;
        case selection_obb:
          return m_obb.translate(T);
          break;
        case selection_sphere:
          return m_sphere.translate(T);
          break;
        case selection_cylinder:
          return m_cylinder.translate(T);
          break;
        case selection_prism:
          return m_prism.translate(T);
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::translate(): case not handled");
          break;
        };
      }

      void rotate(matrix3x3_type const & R)
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.rotate(R);
          break;
        case selection_obb:
          return m_obb.rotate(R);
          break;
        case selection_sphere:
          return m_sphere.rotate(R);
          break;
        case selection_cylinder:
          return m_cylinder.rotate(R);
          break;
        case selection_prism:
          return m_prism.rotate(R);
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::rotate(): case not handled");
          break;
        };
      }

      void scale(real_type const & s)
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.scale(s);
          break;
        case selection_obb:
          return m_obb.scale(s);
          break;
        case selection_sphere:
          return m_sphere.scale(s);
          break;
        case selection_cylinder:
          return m_cylinder.scale(s);
          break;
        case selection_prism:
          return m_prism.scale(s);
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::scale(): case not handled");
          break;
        };
      }

      vector3_type get_support_point(vector3_type const & v) const
      {
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.get_support_point(v);
          break;
        case selection_obb:
          return m_obb.get_support_point(v);
          break;
        case selection_sphere:
          return m_sphere.get_support_point(v);
          break;
        case selection_cylinder:
          return m_cylinder.get_support_point(v);
          break;
        case selection_prism:
          return m_prism.get_support_point(v);
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::get_support_point(): case not handled");
          break;
        };
        return vector3_type(value_traits::zero(),value_traits::zero(),value_traits::zero());
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
        switch(m_picked)
        {
        case selection_aabb:
          return m_aabb.compute_collision_aabb(r,R,min_coord,max_coord);
          break;
        case selection_obb:
          return m_obb.compute_collision_aabb(r,R,min_coord,max_coord);
          break;
        case selection_sphere:
          return m_sphere.compute_collision_aabb(r,R,min_coord,max_coord);
          break;
        case selection_cylinder:
          return m_cylinder.compute_collision_aabb(r,R,min_coord,max_coord);
          break;
        case selection_prism:
          return m_prism.compute_collision_aabb(r,R,min_coord,max_coord);
          break;
        case selection_tetrahedron:
        case selection_undefined:
          assert(!"Hybrid::compute_collision_aabb(): case not handled");
          break;
        };
      }

    };

  }  // namespace geometry

} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_HYBRID_H
#endif
