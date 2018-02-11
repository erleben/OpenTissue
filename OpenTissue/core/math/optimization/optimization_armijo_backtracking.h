#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_ARMIJO_BACKTRACKING_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_ARMIJO_BACKTRACKING_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_constants.h>
#include <OpenTissue/core/math/optimization/optimization_stagnation.h>
#include <OpenTissue/core/math/optimization/optimization_relative_convergence.h>

#include <stdexcept>
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {
      /**
      * Armijo Back-tracking Line-Search.
      *
      * We can think of f(x) as being a function of the step-length parameter, tau, thus we may write
      *
      *  f( tau ) = f ( x + tau dx )
      *
      * A first order Taylor approximation around tau=0 yields
      *
      *    f( tau ) ~ f(0) + f'(0) tau
      *
      * The sufficient decrease conditition (the Armijo condition) is
      *
      *    f( tau ) < f(0) + alpha f'(0) tau
      *
      * for some alpha in ]0..1]. Observe that
      *
      *   f' = d/d tau f(x + tau dx) = nabla f(x)^T dx
      *
      * This is nothing more than the directional derivative of f
      * taken at x and in the direction of dx.If we write
      *
      *   gamma = alpha*nabla f(x)^T dx 
      *
      * then we can rephrase the test as follows
      *
      *   f( x + tau dx ) < f(x) + gamma tau
      *
      * This is the Armijo test used in the un-projected line-search
      * performed by this function.
      *
      *
      * @param f                     The function functor. This is used for computing the value of f(x+tau*dx).
      * @param nabla_f               The value of df/dx at the current iterate. That is the gradient value at the iterate value x.
      * @param x                     The current iterate value.
      * @param x_tau                 Upon return this argument holds the value of, x + \tau*dx, the new iterate.
      * @param dx                    The descent direction along which the line-search is performed.
      * @param relative_tolerance    This argument holds the value used in the relative stopping criteria.
      *                              Setting the value to zero will make the test in-effective.
      * @param stagnation_tolerance  This argument holds the value used in the stagnation test. It is
      *                              an upper bound of the infinity-norm of the difference in the x-solution
      *                              between two iterations.  Setting the value to zero will make the test in-effective.
      * @param alpha                 Armijo test paramter, should be in the range 0..1, this is the fraction of sufficient decrease that is needed by the line-search method. A good value is often 0.00001;
      * @param beta                  The step-length reduction parameter. Everytime the Armijo condition fails then the step length is reduced by this fraction. Usually alpha < beta < 1. A good value is often 0.5;
      * @param f_tau                 Upon invokation this argument should hold the value of f(x) upon return the value is set equal to f(x+tau*dx).
      * @param status                Upon return this argument holds the status of the line-search: OK = 0, NON_DESCENT_DIRECTION = 1, BACKTRACKING_FAILED = 2, STAGNATION = 3, RELATIVE_CONVERGENCE = 4.
      * @return                      The step-length tau.
      */
      template < typename T, typename function_type >
      inline T armijo_backtracking(
        function_type const & f
        , ublas::vector<T> const & nabla_f
        , ublas::vector<T> const & x
        , ublas::vector<T> & x_tau
        , ublas::vector<T> const & dx
        , T      const & relative_tolerance
        , T      const & stagnation_tolerance
        , T      const & alpha
        , T      const & beta
        , T            & f_tau
        , size_t       & status
        )
      {
        using std::fabs;
        using std::max;

        typedef OpenTissue::math::ValueTraits<T>  value_traits;

        T const TOO_TINY = ::boost::numeric_cast<T>(0.00001);

        status = OK;

        if(relative_tolerance < value_traits::zero() )
          throw std::invalid_argument("relative_tolerance must be non-negative");
        if(stagnation_tolerance < value_traits::zero() )
          throw std::invalid_argument("stagnation_tolerance must be non-negative");
        if (beta >= value_traits::one() )
          throw std::invalid_argument("Illegal beta value");
        if (alpha <= value_traits::zero() )
          throw std::invalid_argument("Illegal alpha value");
        if(beta<=alpha)
          throw std::invalid_argument("beta must be larger than alpha");

        size_t const m = x.size();

        // Test if we have a problem to work on
        if(m==0)
          return value_traits::zero();

        assert(x.size() == x_tau.size() || !"armijo_backtracking(): x and x_tau have incompatible dimensions");

        // Do line-search by performing Armijo back-tracking
        T const f_0 = f_tau;
        T       tau = value_traits::one();
        x_tau       = x + dx;
        f_tau       = f(x_tau);

        assert( is_number( f_tau ) || !"armijo_backtracking(): internal error, NAN is encountered?");

        T gamma = alpha*ublas::inner_prod( nabla_f, dx );

        assert( is_number( gamma ) || !"armijo_backtracking(): internal error, NAN is encountered?");

        // Test if we have non descent direction
        if( gamma >= value_traits::zero() )
        {
          status = NON_DESCENT_DIRECTION;
          f_tau = f_0;
          x_tau.assign( x );
          return value_traits::zero();
        }

        // Perform back-tracking line-search
        while( ( f_tau > (f_0 + gamma*tau ) ) && tau > TOO_TINY )
        {
          tau *= beta;
          assert( is_number( tau ) || !"armijo_backtracking(): internal error, NAN is encountered?");
          x_tau = x + dx*tau;
          f_tau = f(x_tau);
          assert( is_number( f_tau ) || !"armijo_backtracking(): internal error, NAN is encountered?");
        }

        // Test if a new step length was computed
        if( (tau < TOO_TINY) && (f_tau > f_0))
          status = BACKTRACKING_FAILED;

        if(stagnation( x, x_tau, stagnation_tolerance ) )
          status = STAGNATION;

        if(relative_convergence(f_0,f_tau, relative_tolerance) )
          status = RELATIVE_CONVERGENCE;

        // Return the new step length to caller
        return tau;
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_ARMIJO_BACKTRACKING_H
#endif
