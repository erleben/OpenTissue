#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PRISM_FIT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PRISM_FIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_plane.h>
#include <OpenTissue/core/containers/mesh/mesh.h>
#include <OpenTissue/core/math/math_eigen_system_decomposition.h>
#include <OpenTissue/core/math/math_covariance.h>
#include <OpenTissue/core/geometry/geometry_graham_scan.h>
#include <OpenTissue/core/math/math_constants.h>


#include <boost/numeric/conversion/cast.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Prism Fit.
    * This computes  a``tight'' fitting enclosing Prism around a given point cloud.
    *
    * Example Usage:
    *
    *    std::vector<vector3<real_type> > points(100);
    *    ... fill in coordinate values of points ...
    *    Prism< ... > prism;
    *    prism_fit(points.begin(),points.end(),prism)
    *
    * Note that you could equally well use it as
    *
    *    vector3<real_type> * points = new vector<vector3<real_type>[100];
    *    ... fill in coordinate values of points ...
    *    Prism< ... > aabb;
    *    prism_fit(points,points+100,prism)
    *
    * @param begin                Iterator to first point
    * @param end                  Iterator to one past the
    *                             last point
    * @param prism                Upon return this argument holds the fitted prism.
    */
    template <typename vector3_iterator,typename prism_type>
    void prism_fit(
      vector3_iterator begin
      , vector3_iterator end
      , prism_type & prism
      , bool const & use_convex_surface = true
      )
    {
      typedef typename prism_type::math_types          math_types;
      typedef typename math_types::real_type           real_type;
      typedef typename math_types::matrix3x3_type      matrix3x3_type;
      typedef typename math_types::vector3_type        vector3_type;
      typedef          geometry::Plane<math_types>     plane_type;

      using std::min;
      using std::max;
      using std::fabs;
      using std::sqrt;
      using boost::numeric_cast;

      assert(begin!=end || !"prism_fit(): beging was equal to end");

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
      real_type s = d(0);
      if(fabs(d(1))<s)
      {
        s = d(1);
        axis = vector3_type(R(0,1),R(1,1),R(2,1));
      }
      if(fabs(d(2))<s)
      {
        s = d(2);
        axis = vector3_type(R(0,2),R(1,2),R(2,2));
      }

      //--- Project points onto plane orthogonal to axis
      real_type lower = math::detail::highest<real_type>();
      real_type upper = math::detail::lowest<real_type>();
      plane_type plane(axis,0.);

      unsigned int count = std::distance(begin,end);
      std::vector<vector3_type> tmp(count);

      vector3_iterator point = begin;
      for(unsigned int i=0; i<count; ++i,++point)
      {
        real_type l = axis * (*point);
        lower = min(lower,l);
        upper = max(upper,l);
        tmp[i] = plane.project((*point)); //---- hmmm, this projection may be un-called for!!!
      }
      real_type height = (upper - lower);

      //--- Find convex hull of projected points
      std::vector<vector3_type> hull;
      graham_scan(tmp.begin(),tmp.end(),hull,plane.n());


      //--- Find an enclosing triangle of the hull
      vector3_type pp,qq,rr;
      vector3_type p,q,r;

      //--- The algorithm at http://www.xore.ca/triangle.php might be better than this...!
      real_type min_area = math::detail::highest<real_type>();
      int N = numeric_cast<int>( hull.size() );
      for(int j=0;j<N;++j)
      {
        int i = (j-1)%N;
        if(i<0)
          i += N;
        int k = (j+1)%N;

        vector3_type u = unit( hull[k]-hull[j] );
        vector3_type v = unit( hull[i]-hull[j] );
        vector3_type bisector = unit(v + u);

        real_type h = -1;
        for(int w=0;w<N;++w)
        {
          vector3_type d1 = hull[w] - hull[j];
          real_type val = bisector * d1;
          if(val>h)
          {
            h = val;
          }
        }
        real_type tu = h / (u * bisector);
        real_type tv = h / (v * bisector);
        pp = hull[j];
        qq = pp + tu*u;
        rr = pp + tv*v;

        u = q - p;
        v = r - p;
        vector3_type uXv = u % v;
        real_type  area  = uXv*uXv; //--- this is actual square twice area!!!
        if(area<min_area)
        {
          min_area = area;
          p = pp;
          q = qq;
          r = rr;
        }
      }

      plane.set(axis,lower);
      p = plane.project(p);
      q = plane.project(q);
      r = plane.project(r);

      prism = prism_type(p,q,r,height);
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_PRISM_FIT_H
#endif
