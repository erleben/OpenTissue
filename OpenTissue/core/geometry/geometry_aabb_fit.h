#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_AABB_FIT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_AABB_FIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cassert>
#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace geometry
  {

    /**
    * AABB Fit.
    * This computes  a``tight'' fitting enclosing AABB around a given point cloud.
    *
    * Example Usage:
    *
    *    std::vector<vector3<real_type> > points(100);
    *    ... fill in coordinate values of points ...
    *    AABB<...> aabb;
    *    aabb_fit(points.begin(),points.end(),aabb)
    *
    * Note that you could equally well use it as
    *
    *    vector3<real_type> * points = new vector<vector3<real_type>[100];
    *    ... fill in coordinate values of points ...
    *    AABB<...> aabb;
    *    aabb_fit(points,points+100,aabb)
    *
    *
    *
    * @param begin                Iterator to first point
    * @param end                  Iterator to one past the
    *                             last point
    * @param aabb                 Upon return this argument holds the fitted AABB.
    */
    template <typename vector3_iterator,typename aabb_type>
    void aabb_fit(vector3_iterator begin, vector3_iterator end,aabb_type & aabb)
    {
      typedef typename aabb_type::real_type  real_type;

      assert(begin!=end || !"aabb_fit(): beging was equal to end");
      aabb.min()(0) = aabb.min()(1)= aabb.min()(2) = math::detail::highest<real_type>();
      aabb.max()(0) = aabb.max()(1)= aabb.max()(2) = math::detail::lowest<real_type>();
      for(vector3_iterator v=begin; v!=end; ++v)
      {
        aabb.min() = min( aabb.min(), *v);
        aabb.max() = max( aabb.max(), *v);
      }
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_AABB_FIT_H
#endif
