#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ELLIPSOID_GROWTH_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ELLIPSOID_GROWTH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/core/geometry/geometry_quadric.h>
#include <cmath>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Ellipsoid Growth.
    *
    * NOTE: Supposed to be used with ellipsoid_growing_fit(...).
    *
    * @param radius    Radius of inner largest empty sphere (S).
    * @param center    Center of sphere S.
    *
    * @param p         A point on the surface of the sphere S.
    * @param q         A point on the surface of the sphere S. Not equal to p.
    *
    * @param begin     Iterator to first point
    * @param end       Iterator to one position past the last point
    *
    * @param r         A third point defining the grown ellipsoid.
    * @param alpha     The alpha value defining the quadric Q = Q1 + alpha Q2.
    */
    template<typename real_type, typename vector3_type, typename vector3_iterator>
    Quadric<real_type> ellipsoid_growth(
      real_type const & radius
      , vector3_type const & center
      , vector3_type const & p
      , vector3_type const & q
      , vector3_iterator begin
      , vector3_iterator end
      , real_type & alpha
      , vector3_type & r
      )
    {
      using std::fabs;

      typedef Quadric<real_type> quadric_type;

      assert(p!=q  || !"ellipsoid_growth(): p and q are the same");
      assert(radius>0 || "ellipsoid_growth(): Non-positive radius");

      vector3_type np = normalize(p - center);
      vector3_type nq = normalize(q - center);

      //--- Defining:
      //---
      //--- (x-p)^T np * (q-x)^T nq = 0           (Hmm, what does this mean about x?) It must lie either on plane (p,np) or (q,nq)...
      //---
      //--- Retwritting using homegeneous coordintes
      //---
      //---          | A2   B2| |x|
      //---  |x^T 1| | B2^T C2| |1|  = 0
      //---
      //--- Which is equivalent to
      //---
      //---     x^T A2 x + 2 B2^T x + C2 = 0
      //---
      //--- Straightforward computation then shows that
      //---
      //--- A2   =    - np nq^T
      //---
      //---           | np0 nq*q + nq0 np*p |
      //--- 2 B2 =    | np1 nq*q + nq1 np*p |
      //---           | np2 nq*q + nq2 np*p |
      //---
      //--- C2 = - p^T np * q^T nq
      //---
      //--- Now defining:
      //---
      //---  (x-c)^T (x-c) - r^2 = 0
      //---
      //--- Then the equivalent expression
      //---
      //---   x^T A1 x + 2 B1^T x + C1 = 0
      //---
      //--- With
      //---
      //---  A1 = I
      //---  B1 = - c
      //---  C1 = c^T c - r^2
      //---
      //--- According to the Ellipsoid::get_equation() we have for a sphere
      //---
      //---  A' = diag(1/r*r)
      //---  B' = - A c
      //---  C' = -c^T B - 1
      //---
      //--- Now by substitution we have
      //---
      //---    x^T A' x + 2 B'^T x + C' = x^T x /r*r  - 2 c^T/ r*r  + c^T c /r*r - 1 = x^T x  - 2 c^T x  + c^T c - r^2
      //---
      //--- Which were the defining equation for the sphere, thus the two quadratic terms are equivalent
      //---
      //----------------------------------------------------------------------------------------
      //--- Define
      //---
      //---  Q1(x)  = x^T A1 x + 2 B1^T x + C1
      //---  Q2(x)  = x^T A2 x + 2 B2^T x + C2
      //---
      //---
      //--- Find new Q'(x) as
      //---
      //---   Q'(x) = Q1(x) + alpha Q2(x)  for alpha > 0
      //---
      //--- That is find maximum
      //---
      //---   alpha(x) = - Q1(x)/Q2(x)  for all Q2(x)<0
      //---
      quadric_type Q1 = make_sphere_quadric(radius,center);
      quadric_type Q2 = make_plane_product_quadric(p, np, q, nq);

      alpha = math::detail::highest<real_type>();
      for(vector3_iterator x = begin;x!=end;++x)
      {
        vector3_type dxp  = (*x) - p;
        vector3_type dxq  = (*x) - q;
        if( (dxp*np<0)  &&  (dxq*nq<0) ) //--- x behind both planes
        {
          real_type q2 = Q2( (*x) );  //--- if x must be behind both plane (p,np) and (q,nq) then Q2 < 0
          real_type q1 = Q1( (*x) );  //--- For all x we have Q1 >= 0
          q1 = q1<0?0:q1;             //--- clamp it to zero inorder to avoid problems with numerical precision
          real_type tst = - q1 / q2;  //--- Solve: Q1 + alpha Q2 = 0 for alpha
          assert(q2 < 0 || !"ellipsoid_growth(): argh something was wrong");
          assert(q1 >= 0 || !"ellipsoid_growth(): argh something was wrong");
          assert(tst >= 0 || !"ellipsoid_growth(): argh something was wrong");
          if(0<=tst && tst < alpha )   //--- See if we have a valid solution, and if alpha is smaller than best known alpha!!!
          {
            alpha = tst;
            r = *x;
          }
        }
      }
      //--- No point x can be used, so ignore alpha
      if( alpha == math::detail::highest<real_type>() )
        alpha=0;

      return (Q1 + (Q2*alpha));
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_ELLIPSOID_GROWTH_H
#endif
