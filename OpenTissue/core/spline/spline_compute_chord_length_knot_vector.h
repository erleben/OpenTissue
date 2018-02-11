#ifndef OPENTISSUE_CORE_SPLINE_COMPUTE_CORD_LENGTH_POLICY_H
#define OPENTISSUE_CORE_SPLINE_COMPUTE_CORD_LENGTH_POLICY_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>

namespace OpenTissue
{
  namespace spline
  {

    /**
    * Compute a knot_vector using chord length parameterization.
    * Creates a knot-vector using the chord length parameterization. Can be used as a functor
    * when doing datapoint interpolation.
    * The operator takes a knot-vector, datapoints and the order k.
    */
    template<typename knot_container, typename point_container>
    inline void compute_chord_length_knot_vector(int const & k, point_container const & X, knot_container & U)
    {
      using std::sqrt;
      using std::fabs;

      typedef typename point_container::value_type       V;
      typedef typename knot_container::value_type        T;
      typedef          OpenTissue::math::ValueTraits<T>  value_traits;

      int i;
      int const n = X.size()+1;

      U.resize( n+k+1 );

      //---First k knots is set to 0 (clamped knot-vector)
      for(i=0; i<k; ++i)
      {
        U[i] = value_traits::zero();
      }

      V diff( X[0].size() );

      T accumulated = value_traits::zero();

      for(i=0; i<=n-3; ++i)
      {
        diff = X[i+1];
        diff -= X[i];

        //--- Use inner_prod function on vectors.
        T length = sqrt( inner_prod(diff,diff) );
        if( fabs(length)> value_traits::zero() )
        {
          accumulated += length;
        }
        else
        {
          accumulated += value_traits::half();
        }
        U[i+4] = accumulated;
      }

      for(i=n+2; i<n+k+1; ++i)
      {
        U[i]=accumulated;
      }

      T const maxU   = (n - value_traits::two() );
      T const scale =  maxU/accumulated;

      for(i=0; i<=n+k; ++i)
      {
        U[i] *= scale;
      }
    }

  } // namespace spline
} // namespace OpenTissue

//OPENTISSUE_CORE_SPLINE_COMPUTE_CORD_LENGTH_POLICY_H
#endif
