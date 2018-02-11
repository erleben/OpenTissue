#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_GROWTH_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_GROWTH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_is_number.h>
#include <OpenTissue/core/math/math_constants.h>

#include <OpenTissue/core/geometry/geometry_quadric.h>
#include <cmath>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Cylindrical Growth.
    * Find largest empty inner ellipsoid by growing it inside a cylindrical tube.
    *
    * NOTE: Supposed to be used with ellipsoid_growing_fit(...).
    *
    * @param center   Center of largest inner empty sphere.
    * @param radius   Radius of largest inner empty sphere.
    * @param p        A point lying on the surface of the inner empty sphere.
    * @param q        Another point lying on the surface of the inner empty sphere.
    * @param r        A third point defining an ellipsoid grown inside the wedge defined by surface points p and q.
    * @param alpha    The weight of the linear combination of ``wedge'' and inner sphere.
    * @param begin    An iterator to the first sample point.
    * @param end      An iterator one past the last sample point.
    *
    * @param beta     Upon return holds the linear weight of the cylindrical quadric, defining the resulting ellipsoid.
    * @param s        Upon return holds the final and fourth point defining the last quadric.
    */
    template<typename real_type, typename vector3_type, typename vector3_iterator>
    Quadric<real_type> cylindrical_growth(
      vector3_type const & center
      , real_type    const & radius
      , vector3_type const & p
      , vector3_type const & q
      , vector3_type const & r
      , real_type    const & alpha
      , vector3_iterator begin
      , vector3_iterator end
      , real_type & beta
      , vector3_type & s
      )
    {
      using std::fabs;
      using std::sqrt;
      using std::max;

      typedef Quadric<real_type> quadric_type;
      assert(p!=q || !"cylindrical_growth(): non-unique point encountered");
      assert(p!=r || !"cylindrical_growth(): non-unique point encountered");
      assert(q!=r || !"cylindrical_growth(): non-unique point encountered");


      vector3_type np = normalize(p - center);
      vector3_type nq = normalize(q - center);

      //---
      //---   Q' = Q1 + alpha Q2
      //---
      //--- Find maximum beta then Q is given by
      //---
      //---     Q = Q' + beta Q3
      //---
      //--- where Q3 is a cylindrical ellipse
      //---
      //--- Compute orthogonal vectors spanning plane given by p,q,r
      //---
      vector3_type u = p;
      vector3_type v = normalize(q-p);
      vector3_type w = normalize( cross( cross(v,r-p) , v ));

      assert(fabs(v*w)<10e-5 || !"cylindrical_growth(): v and w were non-orthogonal");

      quadric_type Q1 = make_sphere_quadric(radius,center);
      quadric_type Q2 = make_plane_product_quadric(p, np, q, nq);
      quadric_type Qprime = Q1 + Q2*alpha;

      //--- Compute the intersection of the plane with the quadric
      //---
      //--- Q(x,y,z) = (x,y,z)^T A (x,y,z) + 2 B^T (x,y,z) + C
      //---
      //--- which gives us an ellipse
      //---
      //---    E(v,w) = (v,w)^T A (v,w) + 2 B^T (v,w) + C
      //---
      //--- in the plane.
      //---
      real_type A00 = v*(Qprime.m_A*v);
      real_type A01 = v*(Qprime.m_A*w);
      real_type A10 = w*(Qprime.m_A*v);
      real_type A11 = w*(Qprime.m_A*w);
      real_type B0  = (Qprime.m_A*u + Qprime.m_B)*v;
      real_type B1  = (Qprime.m_A*u + Qprime.m_B)*w;
      real_type C = u*(Qprime.m_A*u) + 2.0*Qprime.m_B*u + Qprime.m_C;

      assert(fabs(A01-A10)<10e-7 || !"cylindrical_growth(): A matrix was non-symmetrical");

      //--- Compute the main axis transform for the ellipse
      //--- By solving eigenvalue problem
      real_type a_coef = 1;
      real_type b_coef = - (A00 + A11);
      real_type c_coef = A00*A11 - A10*A01;
      real_type d = max(0.,b_coef*b_coef - 4*a_coef*c_coef);
      d = sqrt(d);
      assert(d>=0  || !"cylinder_growth(): d was negative");

      real_type eig0;
      if(b_coef>0)
        eig0 = (-b_coef - d)/(2.0*a_coef);
      else
        eig0 = (-b_coef + d)/(2.0*a_coef);
      real_type eig1 = c_coef / (a_coef*eig0);

      assert(is_number(eig0) || !"cylinder_growth(): eigenvalue was nan");
      assert(is_number(eig1) || !"cylinder_growth(): eigenvalue was nan");
      assert(eig0!=0         || !"cylinder_growth(): eigenvalue was zero");
      assert(eig1!=0         || !"cylinder_growth(): eigenvalue was zero");

      //--- Solve eigenvectors by solving: (A-I*eig0) = 0
      real_type vec0_0,vec0_1,vec1_0,vec1_1,_A00,_A11;

      _A00 = (A00-eig0);
      _A11 = (A11-eig0);

      if(fabs(_A00)>0 && fabs(_A11))
      {

        assert( _A00 || _A11   || !"cylinder_growth(): A00 and A11 were both zero");
        if(fabs(_A00) > fabs(_A11))
        {
          vec0_0 =  - A01 / _A00;
          vec0_1 =  1;
        }
        else
        {
          vec0_0 =  1;
          vec0_1 =  - A10 / _A11;
        }
        assert(is_number(vec0_0) || !"cylinder_growth(): eigenvector coordinate was nan");
        assert(is_number(vec0_1) || !"cylinder_growth(): eigenvector coordinate was nan");
        real_type lgh_vec0 = sqrt(vec0_0*vec0_0 + vec0_1*vec0_1);
        assert(lgh_vec0>0);
        vec0_0 /= lgh_vec0;
        vec0_1 /= lgh_vec0;
        assert(is_number(vec0_0) || !"cylinder_growth(): eigenvector coordinate was nan");
        assert(is_number(vec0_1) || !"cylinder_growth(): eigenvector coordinate was nan");

      }
      else
      {
        vec0_0 =  1;
        vec0_1 =  0;
      }
      //--- Solve eigenvectors by solving: (A-I*eig1) = 0
      //--- Hmmm, we only have two eigenvectors and they are orthogonal, so
      //--- in 2D we get the other by using the hat-operation...
      vec1_0 = - vec0_1; 
      vec1_1 =   vec0_0;



      //--- Transform everything into (x,y,z) space and
      //--- compute midpoint and main axes for the ellipse.
      real_type detA = A00*A11 - A10*A01;  //--- Using cramers rule to solve   A c = - B
      real_type detA0 = -B0*A11 + B1*A01;
      real_type detA1 = -A00*B1 + A10*B0;
      real_type c0 = detA0/detA;
      real_type c1 = detA1/detA;
      assert(is_number(c0) || !"cylinder_growth(): minpoint coordinate was nan" );
      assert(is_number(c1) || !"cylinder_growth(): minpoint coordinate was nan" );

      real_type dn = c0*(A00*c0+A01*c1) +  c1*(A10*c0+A11*c1) + 2.0* c0*B0+c1*B1 + C;
      real_type radius0 = sqrt(-dn/eig0);
      real_type radius1 = sqrt(-dn/eig1);

      assert(is_number(radius0)  || !"cylinder_growth(): radius 0 was nan" );
      assert(is_number(radius1)  || !"cylinder_growth(): radius 1 was nan" );
      assert(radius0>0           || !"cylinder_growth(): radius 0 was negative" );
      assert(radius1>0           || !"cylinder_growth(): radius 1 was negative" );



      vector3_type ellipse_center = u +  v*c0 + w*c1;
      vector3_type axis0 = normalize(v*vec0_0 + w*vec0_1);
      vector3_type axis1 = normalize(v*vec1_0 + w*vec1_1);

      quadric_type Q3 = make_cylinder_quadric(ellipse_center,axis0,radius0,axis1,radius1);
      beta = math::detail::highest<real_type>();
      for(vector3_iterator x = begin;x!=end;++x)
      {
        vector3_type dxp = (*x - p);
        if((dxp*np < 0))
        {
          real_type q3 = Q3( (*x) );
          if (q3 < 0)
          {
            real_type q12 = - Qprime( (*x) );
            assert(q12 <= 0 || !"cylinder_growth(): Q1 + alpha Q2 was non-empty");
            real_type tst = - q12 / q3;
            if( 0 <= tst && tst < beta )
            {
              beta = tst;
              s = (*x);
            }
          }
        }
      }
      if( beta == math::detail::highest<real_type>() )
        beta = 0;

      return (Q1 + (Q2*alpha) + (Q3*beta));
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_CYLINDER_GROWTH_H
#endif
