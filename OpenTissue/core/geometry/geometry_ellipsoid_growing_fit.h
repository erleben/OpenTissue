#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_GROWING_ELLIPSOID_FIT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_GROWING_ELLIPSOID_FIT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/geometry/geometry_ellipsoid.h>
#include <OpenTissue/core/geometry/geometry_quadric.h>
#include <OpenTissue/core/geometry/geometry_spherical_growth.h>
#include <OpenTissue/core/geometry/geometry_cylinder_growth.h>
#include <OpenTissue/core/geometry/geometry_ellipsoid_growth.h>
#include <cmath>

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Fitting by growing Ellipsoid.
    * Determines ``largest'' possible (inner) empty ellipsoid by a growing strategy. 
    *
    *
    * @param p       The surface point, used to set the starting position of the growing ellipsoid.
    * @param n       The ``outward'' pointing normal at the surface point p.
    * @param begin   An iterator to the first sample point.
    * @param end     An iterator one past the last sample point.
    *
    * @return        The fitted ellipsoid.
    */
    template<typename vector3_type, typename vector3_iterator, typename ellipsoid_type>
    void ellipsoid_growth_fit(
      vector3_type const & p
      , vector3_type const & n
      , vector3_iterator begin
      , vector3_iterator end
      , ellipsoid_type & E
      )
    {

      using std::fabs;

      typedef typename vector3_type::value_type             real_type;
      typedef Quadric<typename vector3_type::value_type>    quadric_type;

      vector3_type q,r,s;
      real_type radius,alpha,beta;

      quadric_type Q1 = spherical_growth(p,n,begin,end,radius,q);
      vector3_type center = p - n*radius;
      quadric_type Q = ellipsoid_growth(radius,center,p,q,begin,end,alpha,r);
      if(!(fabs(alpha)>0))//--- Hmmm: alpha==0
      {
        std::cout << "alpha == 0" << std::endl;
        E.set_equation(Q1.m_A, Q1.m_B, Q1.m_C);
        return;
      }
      if(!is_valid_ellipsoid(Q))
      {
        E.set_equation(Q1.m_A, Q1.m_B, Q1.m_C);
        return;
      }
      quadric_type Q4 = cylindrical_growth(center,radius,p,q,r,alpha,begin,end,beta,s);
      if(!(fabs(beta)>0))//--- Hmmm: beta==0
      {
        std::cout << "beta == 0" << std::endl;
        E.set_equation(Q.m_A, Q.m_B, Q.m_C);
        return;
      }
      //if(!is_valid_ellipsoid(Q4))
      //{
      //  std::cout << "invalid ellipsoid" << std::endl;
      //  E.set_equation(Q.m_A, Q.m_B, Q.m_C);
      //  return;
      //}
      E.set_equation(Q4.m_A, Q4.m_B, Q4.m_C);
      return;
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_GROWING_ELLIPSOID_FIT_H
#endif
