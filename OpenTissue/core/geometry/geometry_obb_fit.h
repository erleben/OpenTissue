#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_FIT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_FIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/math/math_covariance.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>

#include <algorithm>
#include <cmath>
#include <iostream>

#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{
  namespace geometry
  {

    /**
    * OBB Covariance Fit.
    * This method uses covariance analysis to fit an good ``tight'' fitting
    * enclosing OBB around a given point cloud.
    *
    * Example Usage:
    *
    *    std::vector<vector3<real_type> > points(100);
    *    ... fill in coordinate values of points ...
    *    OBB<real_type> obb;
    *    obb_fit(points.begin(),points.end(),obb)
    *
    * Note that you could equally well use it as
    *
    *    vector3<real_type> * points = new vector<vector3<real_type>[100];
    *    ... fill in coordinate values of points ...
    *    OBB<real_type> obb;
    *    obb_fit(points,points+100,obb)
    *
    *
    *
    * @param begin                Iterator to first point
    * @param end                  Iterator to one past the
    *                             last point
    * @param obb                  Upon return this argument holds the fitted OBB.
    * @param use_convex_surface   If this argument is set to true, the covariace of
    *                             the surface of the convex hull of the point cloud
    *                             is used to extact axis information. Default value
    *                             is true.
    */
    template <typename vector3_iterator,typename obb_type>
    void obb_fit(
      vector3_iterator begin
      , vector3_iterator end
      , obb_type & obb
      , bool const & use_convex_surface = true
      )
    {
      typedef typename obb_type::math_types        math_types;
      typedef typename math_types::real_type       real_type;
      typedef typename math_types::vector3_type    vector3_type;
      typedef typename math_types::matrix3x3_type  matrix3x3_type;

      assert(begin!=end || !"obb_fit(): beging was equal to end");

      matrix3x3_type cov;
      vector3_type mean;
      if(use_convex_surface)
      {
        polymesh::PolyMesh<math_types> mesh;
        mesh::convex_hull(begin, end,mesh);
        mesh::compute_surface_covariance(mesh,mean,cov);
      }
      else
      {
        math::covariance(begin, end, mean, cov);
      }
      matrix3x3_type R;
      vector3_type d;
      math::eigen(cov,R,d);

      vector3_type v1 = vector3_type(R(0,0),R(1,0),R(2,0));
      vector3_type v2 = vector3_type(R(0,1),R(1,1),R(2,1));
      vector3_type v3 = vector3_type(R(0,2),R(1,2),R(2,2));
      vector3_type v1Xv2 = v1 % v2;
      if((v1Xv2 * v3)<0)
      {
        R(0,0) =  v1(0);      R(1,0) =  v1(1);      R(2,0) =  v1(2);
        R(0,1) =  v2(0);      R(1,1) =  v2(1);      R(2,1) =  v2(2);
        R(0,2) = -v3(0);      R(1,2) = -v3(1);      R(2,2) = -v3(2);
      }
      vector3_type min_coord;
      vector3_type max_coord;
      min_coord(0) = min_coord(1)= min_coord(2) = math::detail::highest<real_type>();
      max_coord(0) = max_coord(1)= max_coord(2) = math::detail::lowest<real_type>();
      real_type tst = 0;
      for(vector3_iterator v=begin; v!=end; ++v)
      {
        tst = v1 * (*v);
        min_coord(0) =  (tst < min_coord(0))? tst : min_coord(0);
        max_coord(0) =  (tst > max_coord(0))? tst : max_coord(0);
        tst = v2 * (*v);
        min_coord(1) =  (tst < min_coord(1))? tst : min_coord(1);
        max_coord(1) =  (tst > max_coord(1))? tst : max_coord(1);
        tst = v3 * (*v);
        min_coord(2) =  (tst < min_coord(2))? tst : min_coord(2);
        max_coord(2) =  (tst > max_coord(2))? tst : max_coord(2);
      }
      vector3_type half_extent = (max_coord - min_coord)*.5;
      vector3_type tmp         = (max_coord + min_coord)*.5;
      vector3_type center      = v1*tmp(0) + v2*tmp(1)  + v3*tmp(2) ;

      obb = obb_type(center,R,half_extent);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_OBB_FIT_H
#endif
