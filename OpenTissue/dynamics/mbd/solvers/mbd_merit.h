#ifndef OPENTISSUE_DYNAMICS_MBD_UTIL_SOLVERS_MBD_MERIT_H
#define OPENTISSUE_DYNAMICS_MBD_UTIL_SOLVERS_MBD_MERIT_H
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
  namespace mbd
  {

    /**
    * Compute merit function and use as a measure of convergence
    *
    *  y = A * x + b
    *
    *   H_i(x) =  min(x_i - l_i, max(x_i - u_i, y_i)).
    *
    *  theta(x) = H(x)^T H(x)/2
    *
    */
    template<typename matrix_type, typename vector_type, typename math_policy>
    typename vector_type::value_type  merit( 
        matrix_type const & A
      , vector_type const & x
      , vector_type const & b
      , vector_type const & lo
      , vector_type const & hi
      , math_policy const & /*tag*/
      )
    {
      using std::min;
      using std::max;

      typedef typename math_policy::value_traits value_traits;
      typedef typename vector_type::size_type    size_type;
      typedef typename vector_type::value_type   real_type;

      vector_type y;
      size_type n;
      math_policy::get_dimension(b,n);
      math_policy::resize(y,n);
      math_policy::prod(A,x,b,y);
      real_type theta = value_traits::zero();
      for (size_type i = 0; i < n; ++ i)
      {
        real_type H_i = min(x(i) - lo(i), max(x(i) - hi(i), y(i) ));
        theta += H_i*H_i;
      }
      theta /= value_traits::two();
      return theta;
    }

  } // namespace mbd
} // namespace OpenTissue

// OPENTISSUE_DYNAMICS_MBD_UTIL_SOLVERS_MBD_MERIT_H
#endif
