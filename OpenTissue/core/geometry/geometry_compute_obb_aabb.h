#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_OBB_AABB_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_OBB_AABB_H
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
    * Compute AABB of OBB.
    *
    *
    * @param position         The position of the center of the OBB.
    * @param R                The orientation of the OBB.
    * @param ext              The half width extent of the OBB.
    * @param min_coord       Upon return holds the minimum coordinate corner of the AABB.
    * @param max_coord       Upon return holds the maximum coordinate corner of the AABB.
    */
    template<typename vector3_type,typename matrix3x3_type>
    void compute_obb_aabb(
      vector3_type const & position
      , matrix3x3_type const & R
      , vector3_type const & ext
      , vector3_type & min_coord
      , vector3_type & max_coord
      )
    {
      using std::fabs;

      typedef typename vector3_type::value_type    real_type;

      // i = ext(0)*(R(0,0),R(1,0),R(2,0))
      // j = ext(1)*(R(0,1),R(1,1),R(2,1))
      // k = ext(2)*(R(0,2),R(1,2),R(2,2))
      real_type xrange = fabs(ext(0)*R(0,0))  + fabs(ext(1)*R(0,1)) + fabs(ext(2)*R(0,2)) ;
      real_type yrange = fabs(ext(0)*R(1,0))  + fabs(ext(1)*R(1,1)) + fabs(ext(2)*R(1,2)) ;
      real_type zrange = fabs(ext(0)*R(2,0))  + fabs(ext(1)*R(2,1)) + fabs(ext(2)*R(2,2)) ;
      min_coord(0) = position(0) - xrange;
      min_coord(1) = position(1) - yrange;
      min_coord(2) = position(2) - zrange;
      max_coord(0) = position(0) + xrange;
      max_coord(1) = position(1) + yrange;
      max_coord(2) = position(2) + zrange;
    }

    /**
    * Compute OBB AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified obb.
    *
    * @param obb        The specified obb.
    * @param aabb       Upon return holds the computed AABB.
    */
    template<typename obb_type,typename aabb_type>
    void compute_obb_aabb(obb_type const & obb,aabb_type & aabb)
    {
      compute_obb_aabb(obb.center(),obb.orientation(),obb.ext(),aabb.min(),aabb.max());
    }

    /**
    * Compute OBB AABB.
    *
    * This function computes a tight fitting axis aligned bounding box around the specified obb.
    *
    * @param obb        The specified obb.
    * @return           The computed AABB.
    */
    template<typename obb_type>
    geometry::AABB<typename obb_type::math_types> compute_obb_aabb(obb_type const & obb)
    {
      geometry::AABB<typename obb_type::math_types> aabb;
      compute_obb_aabb(obb,aabb);
      return aabb;
    }

  } //End of namespace geometry
} //End of namespace OpenTissue

// OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_OBB_AABB_H
#endif
