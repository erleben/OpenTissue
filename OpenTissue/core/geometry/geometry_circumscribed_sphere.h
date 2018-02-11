#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CIRCUMSCRIBED_SPHERE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CIRCUMSCRIBED_SPHERE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>
#include <OpenTissue/core/math/math_precision.h>

#include <cmath>
#include <cassert>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    *
    * @param p
    *
    * @return
    */
    template<typename vector3_type, typename sphere_type>
    void compute_circumscribed_sphere(vector3_type const & p, sphere_type & sphere)
    {
      sphere = sphere_type(p, 0.0);
    }

    /**
    *
    * @param p0
    * @param p1
    *
    * @return
    */
    template<typename vector3_type, typename sphere_type>
    void compute_circumscribed_sphere(vector3_type const & p0,vector3_type const & p1, sphere_type & sphere)
    {
      using std::sqrt;
      vector3_type d = p1 - p0;
      sphere = sphere_type((p0+p1)*.5,  sqrt( d*d )*.5  );
    }

    /**
    *
    * @param p0
    * @param p1
    * @param p2
    *
    * @return
    */
    template<typename vector3_type, typename sphere_type>
    void compute_circumscribed_sphere(vector3_type const & p0, vector3_type const & p1, vector3_type const & p2, sphere_type & sphere)
    {
      using std::fabs;
      using std::sqrt;

      typedef typename vector3_type::value_type real_type;

      static real_type const epsilon = math::working_precision<real_type>();

      // Compute the circle (in 3D) containing p0, p1, and p2.  The center in
      // barycentric coordinates is C = u0*p0+u1*p1+u2*p2 where u0+u1+u2=1.
      // The center is equidistant from the three points, so |C-p0| = |C-p1| =
      // |C-p2| = R where R is the radius of the circle.
      //
      // From these conditions,
      //   C-p0 = u0*A + u1*B - A
      //   C-p1 = u0*A + u1*B - B
      //   C-p2 = u0*A + u1*B
      // where A = p0-p2 and B = p1-p2, which leads to
      //   r^2 = |u0*A+u1*B|^2 - 2*Dot(A,u0*A+u1*B) + |A|^2
      //   r^2 = |u0*A+u1*B|^2 - 2*Dot(B,u0*A+u1*B) + |B|^2
      //   r^2 = |u0*A+u1*B|^2
      // Subtracting the last equation from the first two and writing
      // the equations as a linear system,
      //
      // +-                 -++   -+       +-        -+
      // | Dot(A,A) Dot(A,B) || u0 | = 0.5 | Dot(A,A) |
      // | Dot(B,A) Dot(B,B) || u1 |       | Dot(B,B) |
      // +-                 -++   -+       +-        -+
      //
      // The following code solves this system for u0 and u1, then
      // evaluates the third equation in r^2 to obtain r.
      vector3_type kA = p0 - p2;
      vector3_type kB = p1 - p2;
      real_type AdA = kA * kA;
      real_type AdB = kA * kB;
      real_type BdB = kB * kB;
      real_type det = AdA*BdB-AdB*AdB;

      if ( fabs(det) > epsilon )
      {
        real_type halfInvDet = 0.5/det;
        real_type u0 = halfInvDet*BdB*(AdA-AdB);
        real_type u1 = halfInvDet*AdA*(BdB-AdB);
        real_type u2 = 1.0-u0-u1;
        vector3_type tmp = u0*kA + u1*kB;
        sphere = sphere_type( u0*p0 + u1*p1 + u2*p2, sqrt(tmp*tmp) );
      }
      else
      {
        //--- This means that all three points are nearly collinear...
        //--- Find to most distant points and generate a sphere!!!
        real_type l0 = length(p1 - p0);
        real_type l1 = length(p2 - p0);
        real_type l2 = length(p2 - p1);
        if(l0 >= l1 && l0>=l2)
          compute_circumscribed_sphere(p0,p1,sphere);
        else if(l1 >= l2 && l1>=l0)
          compute_circumscribed_sphere(p0,p2,sphere);
        else if(l2 >= l1 && l2>=l0)
          compute_circumscribed_sphere(p1,p2,sphere);
        //else
        //  sphere = sphere_type( vector3_type(0,0,0), math::detail::highest<typename sphere_type::real_type>() );
      }
    }

    /**
    *
    * @param p0
    * @param p1
    * @param p2
    * @param p3
    *
    * @return
    */
    template<typename vector3_type, typename sphere_type>
    void compute_circumscribed_sphere(vector3_type const & p0, vector3_type const & p1, vector3_type const & p2, vector3_type const & p3, sphere_type & sphere)
    {
      using std::fabs;
      using std::sqrt;
      typedef typename vector3_type::value_type real_type;
      static real_type epsilon = math::working_precision<real_type>();
      vector3_type kE10 = p0 - p3;
      vector3_type kE20 = p1 - p3;
      vector3_type kE30 = p2 - p3;
      real_type A[3][3];
      A[0][0] = kE10 * kE10;
      A[0][1] = kE10 * kE20;
      A[0][2] = kE10 * kE30;
      A[1][0] = A[0][1];
      A[1][1] = kE20 * kE20;
      A[1][2] = kE20 * kE30;
      A[2][0] = A[0][2];
      A[2][1] = A[1][2];
      A[2][2] = kE30 * kE30;
      real_type B[3];
      B[0] = 0.5*A[0][0];
      B[1] = 0.5*A[1][1];
      B[2] = 0.5*A[2][2];
      real_type invA[3][3];
      invA[0][0] = A[1][1]*A[2][2] - A[1][2]*A[2][1];
      invA[0][1] = A[0][2]*A[2][1] - A[0][1]*A[2][2];
      invA[0][2] = A[0][1]*A[1][2] - A[0][2]*A[1][1];
      invA[1][0] = A[1][2]*A[2][0] - A[1][0]*A[2][2];
      invA[1][1] = A[0][0]*A[2][2] - A[0][2]*A[2][0];
      invA[1][2] = A[0][2]*A[1][0] - A[0][0]*A[1][2];
      invA[2][0] = A[1][0]*A[2][1] - A[1][1]*A[2][0];
      invA[2][1] = A[0][1]*A[2][0] - A[0][0]*A[2][1];
      invA[2][2] = A[0][0]*A[1][1] - A[0][1]*A[1][0];

      real_type det3 = A[0][0]*invA[0][0] + A[0][1]*invA[1][0] + A[0][2]*invA[2][0];

      if ( fabs(det3) > epsilon )
      {
        real_type inv_det = 1.0/det3;

        int row, col;
        for (row = 0; row < 3; ++row)
        {
          for (col = 0; col < 3; ++col)
            invA[row][col] *= inv_det;
        }

        real_type U[4];
        for (row = 0; row < 3; ++row)
        {
          U[row] = 0.0f;
          for (col = 0; col < 3; ++col)
            U[row] += invA[row][col]*B[col];
        }

        U[3] = 1.0 - U[0] - U[1] - U[2];
        vector3_type tmp = U[0]*kE10 + U[1]*kE20 + U[2]*kE30;
        sphere = sphere_type( U[0]*p0 + U[1]*p1 + U[2]*p2 +  U[3]*p3, sqrt(tmp*tmp) );
      }
      else
      {
        //--- This means that all four points are nearly planar, so set U_2 = 0! and use pseudo-inverse!!!
        real_type PA[2][2]; 
        real_type PB[2];
        PA[0][0] = A[0][0]*A[0][0] + A[1][0]*A[1][0] +  A[2][0]*A[2][0];
        PA[0][1] = A[0][0]*A[0][1] + A[1][0]*A[1][1] +  A[2][0]*A[2][1];
        PB[0]    = A[0][0]*B[0]    + A[1][0]*B[1]    +  A[2][0]*B[2];
        PA[1][0] = A[0][1]*A[0][0] + A[1][1]*A[1][0] +  A[2][1]*A[2][0];
        PA[1][1] = A[0][1]*A[0][1] + A[1][1]*A[1][1] +  A[2][1]*A[2][1];
        PB[1]    = A[0][1]*B[0]    + A[1][1]*B[1]    +  A[2][1]*B[2];
        real_type det = PA[0][0]*PA[1][1] - PA[1][0]*PA[0][1];

        if(fabs(det) > epsilon)
        {
          real_type U[4];
          real_type inv_det = 1.0/det;
          U[0] = inv_det*( PB[0]*PA[1][1] - PB[1]*PA[0][1] ); //--- Cramers Rule...
          U[1] = inv_det*( PA[0][0]*PB[1] - PA[1][0]*PB[0] );
          U[2] = 0;
          U[3] = 1.0 - U[0] - U[1] - U[2];
          vector3_type tmp = U[0]*kE10 + U[1]*kE20 + U[2]*kE30;
          sphere = sphere_type( U[0]*p0 + U[1]*p1 + U[2]*p2 +  U[3]*p3, sqrt(tmp*tmp) );
        }
        else
        {
          //--- This means that all three points are nearly collinear...
          //--- Find to most distant points and generate a sphere!!!
          vector3_type m[4];
          m[0] = p0;      m[1] = p1;      m[2] = p2;      m[3] = p3;
          real_type sqr_dist = 0;
          int s=0,t=0;
          for(int i=0;i<3;++i)
            for(int j=i+1;j<3;++j)
            {
              vector3_type d = m[i]-m[j];
              real_type tmp = d*d;
              if(tmp>sqr_dist)
              {
                sqr_dist = tmp;
                s = i;
                t = j;
              }
            }
            compute_circumscribed_sphere(m[s],m[t],sphere);
            //sphere = sphere_type( vector3_type(0,0,0), math::detail::highest<typename sphere_type::real_type>() );
        }
      }
      //sphere = sphere_type(vector3_type(0,0,0), math::detail::highest<typename sphere_type::real_type>() );
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CIRCUMSCRIBED_SPHERE_H
#endif
