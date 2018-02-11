#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_PLANE_AABB_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_PLANE_AABB_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/core/geometry/geometry_aabb.h>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Compute AABB of plane.
    *
    * @param n              The plane normal.
    * @param w              The ortogonal distance from origo.
    * @param min_coord       Upon return holds the minimum coordinate corner of the AABB.
    * @param max_coord       Upon return holds the maximum coordinate corner of the AABB.
    */
    template<typename vector3_type,typename real_type>
    void compute_plane_aabb(vector3_type const & n,real_type const & w,vector3_type & min_coord,vector3_type & max_coord)
    {
      real_type thickness = 10.;
      if(n(0)==0 && n(1)==0 )
      {
        min_coord(0) = math::detail::lowest<real_type>();
        min_coord(1) = math::detail::lowest<real_type>();
        max_coord(0) = math::detail::highest<real_type>();
        max_coord(1) = math::detail::highest<real_type>();
        min_coord(2) =   w - thickness;
        max_coord(2) =   w + thickness;
      }
      else if(n(0)==0 && n(2)==0 )
      {
        min_coord(0) = math::detail::lowest<real_type>();
        min_coord(2) = math::detail::lowest<real_type>();
        max_coord(0) = math::detail::highest<real_type>();
        max_coord(2) = math::detail::highest<real_type>();
        min_coord(1) =   w - thickness;
        max_coord(1) =   w + thickness;
      }
      else if(n(1)==0 && n(2)==0 )
      {
        min_coord(1) = math::detail::lowest<real_type>();
        min_coord(2) = math::detail::lowest<real_type>();
        max_coord(1) = math::detail::highest<real_type>();
        max_coord(2) = math::detail::highest<real_type>();
        min_coord(0) =   w - thickness;
        max_coord(0) =   w + thickness;
      }
      else
      {
        min_coord(0) = math::detail::lowest<real_type>();
        min_coord(1) = math::detail::lowest<real_type>();
        min_coord(2) = math::detail::lowest<real_type>();
        max_coord(0) = math::detail::highest<real_type>();
        max_coord(1) = math::detail::highest<real_type>();
        max_coord(2) = math::detail::highest<real_type>();
      }
    }

    /**
    * Compute Plane AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified plane.
    *
    * @param plane        The specified plane.
    * @param aabb       Upon return holds the computed AABB.
    */
    template<typename plane_type,typename aabb_type>
    void compute_plane_aabb(plane_type const & plane,aabb_type & aabb)
    {
      compute_plane_aabb(plane.n(),plane.w(),aabb.min(),aabb.max());
    }

    /**
    * Compute Plane AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified plane.
    *
    * @param plane      The specified plane.
    * @return           The computed AABB.
    */
    template<typename plane_type>
    geometry::AABB<typename plane_type::math_types> compute_plane_aabb(plane_type const & plane)
    {
      geometry::AABB<typename plane_type::math_types> aabb;
      compute_plane_aabb(plane,aabb);
      return aabb;
    }

  } //End of namespace geometry
} //End of namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_PLANE_AABB_H
#endif
