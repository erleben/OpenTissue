#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_HYBRID_FIT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_HYBRID_FIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_hybrid.h>
#include <OpenTissue/core/geometry/geometry_aabb_fit.h>
#include <OpenTissue/core/geometry/geometry_obb_fit.h>
#include <OpenTissue/core/geometry/geometry_compute_smallest_sphere.h>
#include <OpenTissue/core/geometry/geometry_cylinder_fit.h>
#include <OpenTissue/core/geometry/geometry_prism_fit.h>

#include <boost/bind.hpp>

#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Hybrid Fit.
    * This computes  a``tight'' fitting enclosing Hybrid around a given point cloud.
    *
    * Example Usage:
    *
    *    std::vector<vector3<real_type> > points(100);
    *    ... fill in coordinate values of points ...
    *    Hybrid<real_type> hybrid;
    *    hybrid_fit(points.begin(),points.end(),hybrid)
    *
    * Note that you could equally well use it as
    *
    *    vector3<real_type> * points = new vector<vector3<real_type>[100];
    *    ... fill in coordinate values of points ...
    *    Hybrid<real_type> hybrid;
    *    hybrid_fit(points,points+100,hybrid)
    *
    *
    *
    * @param begin                Iterator to first point
    * @param end                  Iterator to one past the
    *                             last point
    * @param hybrid               Upon return this argument holds the fitted Hybrid.
    * @param use_convex_surface   If this argument is set to true, the covariace of
    *                             the surface of the convex hull of the point cloud
    *                             is used to extact axis information. Default value
    *                             is true.
    */
    template <typename vector3_iterator,typename hybrid_type>
    void hybrid_fit(
      vector3_iterator begin
      , vector3_iterator end
      , hybrid_type & hybrid
      , bool const & use_convex_surface = true
      )
    {
      typedef typename hybrid_type::real_type        real_type;
      typedef typename hybrid_type::matrix3x3_type   matrix3x3_type;
      typedef typename hybrid_type::vector3_type     vector3_type;

      //assert(begin!=end || !"hybrid_fit(): begin was equal to end");
      if(begin==end)
        return;

      aabb_fit(                begin, end, hybrid.m_aabb                         );
      obb_fit(                 begin, end ,hybrid.m_obb,      use_convex_surface );
      compute_smallest_sphere( begin, end, hybrid.m_sphere                       );
      cylinder_fit(            begin, end, hybrid.m_cylinder, use_convex_surface );
      prism_fit(               begin, end, hybrid.m_prism,    use_convex_surface);

      real_type V = hybrid.m_aabb.volume();
      hybrid.m_picked = hybrid_type::selection_aabb;

      real_type obbV = hybrid.m_obb.volume();
      if(obbV<V)
      {
        V = obbV;
        hybrid.m_picked = hybrid_type::selection_obb;
      }
      real_type cylV = hybrid.m_cylinder.volume();
      if(cylV<V)
      {
        V = cylV;
        hybrid.m_picked = hybrid_type::selection_cylinder;
      }
      real_type sphV = hybrid.m_sphere.volume();
      if(sphV<V)
      {
        V = sphV;
        hybrid.m_picked = hybrid_type::selection_sphere;
      }
      real_type prmV = hybrid.m_prism.volume();
      if(prmV<V)
      {
        V = prmV;
        hybrid.m_picked = hybrid_type::selection_prism;
      }
    }

    /**
    * Fit a hybrid to enclose a collection of volumes/hybrids.
    *
    * The collection could be hybrid instances, or pointers to hybrids
    * or volume adapters etc.. 
    *
    * @param begin    An iterator to a hybrid, pointer volume adapter, etc..
    * @param end      An iterator to a hybrid, pointer volume adapter, etc..
    * @param hybrid   Upon return contains the resulting hybrid.
    */
    template <typename hybrid_volume_iterator,typename hybrid_type>
    void hybrid_volume_fit(
      hybrid_volume_iterator begin
      , hybrid_volume_iterator end
      , hybrid_type & hybrid
      )
    {
      typedef typename hybrid_type::volume_type          volume_type;
      typedef typename hybrid_type::vector3_type         vector3_type;

      //assert(begin!=end || !"hybrid_volume_fit(): begin was equal to end");
      if(begin==end)
        return;

      std::vector<vector3_type> points;

      std::for_each( 
        begin
        , end
        , boost::bind( 
        &volume_type::compute_surface_points
        , _1
        , boost::ref( points ) 
        )
        );

      //std::for_each( 
      //    points.begin()
      //  , points.end()
      //  , std::cout << boost::lambda::_1 
      //  );
      //std::cout << std::endl;

      hybrid_fit(points.begin(),points.end(),hybrid,true);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_HYBRID_FIT_H
#endif
