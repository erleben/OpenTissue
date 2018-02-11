#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SPHERE_AABB_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SPHERE_AABB_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_aabb.h>

namespace OpenTissue
{
  namespace geometry
  {
    /**
    * Compute Sphere AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified sphere.
    *
    * @param center          The center of the sphere.
    * @param radius          The radius of the sphere.
    * @param min_coord       Upon return holds the minimum coordinate corner of the AABB.
    * @param max_coord       Upon return holds the maximum coordinate corner of the AABB.
    */
    template<typename vector3_type,typename real_type>
    void compute_sphere_aabb(vector3_type const & center,real_type const & radius,vector3_type & min_coord,vector3_type & max_coord)
    {
      min_coord(0) = center(0) - radius;
      min_coord(1) = center(1) - radius;
      min_coord(2) = center(2) - radius;
      max_coord(0) = center(0) + radius;
      max_coord(1) = center(1) + radius;
      max_coord(2) = center(2) + radius;
    }

    /**
    * Compute Sphere AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified sphere.
    *
    * @param sphere     The specified sphere.
    * @param aabb       Upon return holds the computed AABB.
    */
    template<typename sphere_type,typename aabb_type>
    void compute_sphere_aabb(sphere_type const & sphere,aabb_type & aabb)
    {
      compute_sphere_aabb(sphere.center(),sphere.radius(),aabb.min(),aabb.max());
    }

    /**
    * Compute Sphere AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified sphere.
    *
    * @param sphere     The specified sphere.
    * @return           The computed AABB.
    */
    template<typename sphere_type>
    geometry::AABB<typename sphere_type::math_types> compute_sphere_aabb(sphere_type const & sphere)
    {
      geometry::AABB<typename sphere_type::math_types> aabb;
      compute_sphere_aabb(sphere,aabb);
      return aabb;
    }

  } //End of namespace geometry
} //End of namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SPHERE_AABB_H
#endif
