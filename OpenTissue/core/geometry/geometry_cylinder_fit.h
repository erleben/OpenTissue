#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_FIT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_FIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>
#include <OpenTissue/core/math/math_covariance.h>

#include <OpenTissue/core/containers/mesh/mesh.h>

#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/geometry/geometry_compute_smallest_sphere.h>

#include <OpenTissue/core/math/math_constants.h>


#include <algorithm>
#include <vector>
#include <cmath>
#include <iostream>


namespace OpenTissue
{
  namespace geometry
  {
    /**
    * Cylinder Covariance Fit.
    * This method uses covariance analysis to fit an good ``tight'' fitting
    * enclosing cylinder around a given point cloud.
    *
    * Example Usage:
    *
    *    std::vector<vector3<real_type> > points(100);
    *    ... fill in coordinate values of points ...
    *    Cylinder< ... > obb;
    *    cylinder_fit(points.begin(),points.end(),obb)
    *
    * Note that you could equally well use it as
    *
    *    vector3<real_type> * points = new vector<vector3<real_type>[100];
    *    ... fill in coordinate values of points ...
    *    Cylinder< ... > obb;
    *    cylinder_fit(points,points+100,obb)
    *
    * @param begin                Iterator to first point
    * @param end                  Iterator to one past the
    *                             last point
    * @param cylinder             Upon return this argument holds the fitted cylinder.
    * @param use_convex_surface   If this argument is set to true, the covariace of
    *                             the surface of the convex hull of the point cloud
    *                             is used to extact axis information. Default value
    *                             is true.
    */
    template <typename vector3_iterator,typename cylinder_type>
    void cylinder_fit(
      vector3_iterator begin
      , vector3_iterator end
      , cylinder_type & cylinder
      , bool const & use_convex_surface = true
      )
    {
      using std::min;
      using std::max;
      using std::fabs;

      typedef typename cylinder_type::math_types  math_types;
      typedef typename math_types::real_type               real_type;
      typedef typename math_types::matrix3x3_type          matrix3x3_type;
      typedef typename math_types::vector3_type            vector3_type;
      typedef geometry::Sphere<math_types>                 sphere_type;

      assert(begin!=end || !"cylinder_fit(): begin was equal to end");

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

      //--- Pick eigenvector with largest eigenvalue as axis of cylinder
      vector3_type axis(R(0,0),R(1,0),R(2,0));
      real_type s = fabs(d(0));
      if(fabs(d(1))>s)
      {
        s = fabs(d(1));
        axis = vector3_type(R(0,1),R(1,1),R(2,1));
      }
      if(fabs(d(2))>s)
      {
        s = fabs(d(2));
        axis = vector3_type(R(0,2),R(1,2),R(2,2));
      }
      //--- Project points onto plane orthogonal to axis
      real_type lower = math::detail::highest<real_type>();
      real_type upper = math::detail::lowest<real_type>();

      geometry::Plane<math_types> plane(axis,0.);
      std::size_t size = std::distance(begin,end);
      std::vector<vector3_type> tmp(size);
      typename std::vector<vector3_type>::iterator p = tmp.begin();
      for(vector3_iterator v=begin; v!=end; ++v)
      {
        real_type l = axis * (*v);
        lower = min(lower,l);
        upper = max(upper,l);
        *p++ = plane.project(*v);
      }
      sphere_type minimal;
      compute_smallest_sphere(tmp.begin(),tmp.end(),minimal);
      real_type radius = minimal.radius();
      real_type height = (upper - lower);
      vector3_type center = minimal.center() + .5*(upper+lower)*axis;
      cylinder = cylinder_type(center,axis,height,radius);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_FIT_H
#endif
