#ifndef OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SIGNED_DISTANCE_TO_TRIANGLE_H
#define OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SIGNED_DISTANCE_TO_TRIANGLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace geometry
  {
    /**
    * Compute Signed Distance to Triangle .
    * This method implicitly assumes that the triangle is non-degenerate!
    *
    * @parma p     A point in space
    * @param pi    First vertex of triangle.
    * @param pj    Second vertex of triangle.
    * @param pk    Third vertex of triangle.
    *
    * @param nv_i  The pseudonormal of vertex i.
    * @param nv_j  The pseudonormal of vertex j.
    * @param nv_k  The pseudonormal of vertex k.
    *
    * @param ne_i  The pseudonormal of the edge going from vertex j to vertex k.
    * @param ne_j  The pseudonormal of the edge going from vertex k to vertex i.
    * @param ne_k  The pseudonormal of the edge going from vertex i to vertex j.
    *
    * @return      The smallest signed distance from p to the specified triangle.
    */
    template<typename vector3_type>
    typename vector3_type::value_type 
      compute_signed_distance_to_triangle(
      vector3_type const & p
      , vector3_type const & pi
      , vector3_type const & pj
      , vector3_type const & pk
      , vector3_type const & nv_i
      , vector3_type const & nv_j
      , vector3_type const & nv_k
      , vector3_type const & ne_i
      , vector3_type const & ne_j
      , vector3_type const & ne_k
      )  
    {
      typedef typename vector3_type::value_type    real_type;
      typedef typename vector3_type::value_traits  value_traits;

      vector3_type ei = unit( pk-pj );
      vector3_type ej = unit( pi-pk );
      vector3_type ek = unit( pj-pi );

      vector3_type n  = - unit( cross( ei, ek ) );
      vector3_type ni = cross( n, ei);
      vector3_type nj = cross( n, ej);
      vector3_type nk = cross( n, ek);

      vector3_type delta_i = ( p-pi );
      vector3_type delta_j = ( p-pj );
      vector3_type delta_k = ( p-pk );

      real_type di = inner_prod(ni,delta_j);
      real_type dj = inner_prod(nj,delta_k);
      real_type dk = inner_prod(nk,delta_i);

      if(di>=value_traits::zero() && dj>=value_traits::zero() && dk>=value_traits::zero())
      {
        return inner_prod(n, delta_i ); 
      }
      else if(di<value_traits::zero())
      {
        real_type tst_j =   inner_prod(ei, delta_j);
        real_type tst_k = - inner_prod(ei, delta_k);
        if(tst_j<value_traits::zero())
        {
          real_type sign = inner_prod(nv_j,delta_j) >=value_traits::zero() ? value_traits::one() : -value_traits::one();
          return sign*length(delta_j);
        }
        if(tst_k<value_traits::zero())
        {
          real_type sign = inner_prod(nv_k,delta_k) >= value_traits::zero() ? value_traits::one() : -value_traits::one();
          return sign*length(delta_k);
        }

        vector3_type orthogonal = delta_j - inner_prod(ei,delta_j)*ei;
        real_type sign = inner_prod(ne_i,orthogonal)>=value_traits::zero() ? value_traits::one() : -value_traits::one();

        return sign*length( orthogonal );
      }
      else if(dj<value_traits::zero())
      {
        real_type tst_k =   inner_prod(ej, delta_k);
        real_type tst_i = - inner_prod(ej, delta_i);
        if(tst_k<value_traits::zero())
        {
          real_type sign = inner_prod(nv_k,delta_k) >=value_traits::zero() ? value_traits::one() : -value_traits::one();
          return sign*length(delta_k);
        }
        if(tst_i<value_traits::zero())
        {
          real_type sign = inner_prod(nv_i,delta_i) >=value_traits::zero() ? value_traits::one() : -value_traits::one();
          return sign*length(delta_i);
        }

        vector3_type orthogonal = delta_k - inner_prod(ej,delta_k)*ej;
        real_type sign = inner_prod(ne_j,orthogonal)>=value_traits::zero() ? value_traits::one() : -value_traits::one();

        return sign*length( orthogonal );
      }
      else if(dk<value_traits::zero())
      {
        real_type tst_i =   inner_prod(ek, delta_i);
        real_type tst_j = - inner_prod(ek, delta_j);
        if(tst_i<value_traits::zero())
        {
          real_type sign = inner_prod(nv_i,delta_i) >=value_traits::zero() ? value_traits::one() : -value_traits::one();
          return sign*length(delta_i);
        }
        if(tst_j<value_traits::zero())
        {
          real_type sign = inner_prod(nv_j,delta_j) >=value_traits::zero() ? value_traits::one() : -value_traits::one();
          return sign*length(delta_j);
        }

        vector3_type orthogonal = delta_i - inner_prod(ek,delta_i)*ek;
        real_type sign = inner_prod(ne_k,orthogonal)>=value_traits::zero() ? value_traits::one() : -value_traits::one();

        return sign*length( orthogonal );
      }

      assert( false || !"compute_signed_distance_to_triangle(): internal error case analysis failed?");
      return value_traits::zero();
    }

  } // namespace geometry
} // namespace OpenTissue

//OPENTISSUE_CORE_GEOMETRY_GEOMETRY_COMPUTE_SIGNED_DISTANCE_TO_TRIANGLE_H
#endif
