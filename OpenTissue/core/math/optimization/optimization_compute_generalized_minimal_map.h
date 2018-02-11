#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_GENERALIZED_MINIMAL_MAP_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_GENERALIZED_MINIMAL_MAP_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <cmath>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

        /**
        * Non-smooth Generalized Minimal Map Reformulation.
        * Compute value of the generalized minimal map non-smooth
        * reformulation of boxed complementarity problem, also
        * known as a mixed complementarity problem (MCP).
        *
        * @param y    This vector contains the value of the function y = f(x).
        * @param l    A functor used to retrieve information about the lower bound function.
        * @param u    A functor used to retrieve information about the upper bound function.
        * @param x    The current value of the x-vector.
        * @param H    Upon return this vector holds the value of H_i(x) =  min(x_i - l_i, max(x_i - u_i, y_i)).
        */
        template < typename T, typename bound_function_type>
        inline void compute_generalized_minimal_map(
          ublas::vector<T> const & y
          , bound_function_type const & l
          , bound_function_type const & u
          , ublas::vector<T> const & x
          , ublas::vector<T> & H
          )  
        {
          using std::min;
          using std::max;

          size_t m = y.size();

          if(m==0)
            return;

          H.resize(m);
          for (size_t i = 0; i < m; ++ i)
          {
            T const l_i = l(x,i);
            T const u_i = u(x,i);
            T const y_i = y(i);
            T const x_i = x(i);
            H(i) = max( x_i - u_i, min( x_i - l_i, y_i ) );
          }
        }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_COMPUTE_GENERALIZED_MINIMAL_MAP_H
#endif
