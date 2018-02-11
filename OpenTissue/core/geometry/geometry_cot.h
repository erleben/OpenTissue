#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COT_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <cmath>   // Needed for std::sqrt and std::fabs

namespace OpenTissue
{
  namespace geometry
  {

    /**
    * Cotangent of two vectors.
    * This implementation is based on the paper:
    *
    *    Meyer, M., Desbrun, M., Schröder, P., AND Barr, A. H. Discrete Differential Geometry Operators for Triangulated 2-Manifolds, 2002. VisMath.
    *
    * This metod computes the cotangent angle between two vectors (pi-p) and (pj-p).
    *
    * Let the angle between the two vectors \vec u and \vec v be \theta. Then by defnition
    *                    
    *    cot(\theta) = \frac{cos \theta}{sin \theta}
    *
    * and we know that
    *
    *  cos(\theta) =  \frac{ u \cdot v  }{ \norm{u} \norm{v} }
    *
    * and
    *
    *  \norm{u \times v} = \norm{u}\norm{v}sin \theta
    *
    * since  1 = cos^2 \theta + sin^2 \theta we re-write
    *
    *   \norm{u}\norm{v}sin \theta  = \norm{u}\norm{v}\left( \pm \sqrt{1 - cos^2 \theta }\right)
    *
    * Knowing that the angle between two vectors is between 0 and \pi, we can throw away
    * the negative solution of sin\theta. Substituting the expression for cos \theta and
    * isolating \sin \theta we get
    *  
    *  sin \theta  = \frac{  \sqrt{\norm{u}^2\norm{v}^2 -  (u \cdot v )^2 } }{\norm{u}\norm{v}}
    *
    * Finally substituting the expressions for cos and sin into the equation for cot we get
    *
    *  cot(\theta)  = \frac{u \codt v}{ \sqrt{\norm{u}^2\norm{v}^2 -  (u \cdot v )^2 } }
    *
    * This is the formula we use for computing cot.
    *
    * @param p      The common tail point of the two vectors
    * @param pi     The head point of one vector.
    * @param pj     The head point of the other vector.
    *
    * @return       The computed value.
    */
    template<typename vector_type>
    typename vector_type::value_type cot(vector_type const & p, vector_type const & pi, vector_type const & pj)  
    {
      using std::sqrt;
      using std::fabs;

      typedef typename vector_type::value_type   real_type;

      static const real_type zero = real_type(); // By standard default constructed integral types are zero

      vector_type u = pi-p;
      vector_type v = pj-p;
      real_type dot = u*v;
      real_type denom = sqrt( (u*u)*(v*v) - dot*dot );

      return (fabs(denom)>zero)? (dot/denom) : zero; 
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COT_H
#endif
