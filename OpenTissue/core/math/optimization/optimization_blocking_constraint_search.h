#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_BLOCKING_CONSTRAINT_SEARCH_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_BLOCKING_CONSTRAINT_SEARCH_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/math_value_traits.h>
#include <OpenTissue/core/math/math_is_number.h>
#include <cmath>
#include <stdexcept>
#include <cassert>

namespace OpenTissue
{
  namespace math
  {
    namespace optimization
    {

      /**
      * Blocking Constraint Search Function.
      * This function tries to determine whether there are some
      * blocking constraints when trying to update the current
      * iterate x along a specified search direction.
      *
      * @param x                The current iterate.
      * @param p                The search direction.
      * @param c                The constraint function, we must have c(x) >= 0
      * @param tau              Upon return this argument holds the step-length that can be taken along the search direction without violating any of the ``linearized'' constraints.
      * @param blocking_idx     Upon return this argument holds the index of one of the blocking constraints (the one with lowest index-value). If no blocking constraint is found then the value is set to one plus the size of the x-vector.
      *
      * @return   If the return value is true then a blocking constraint was found.
      */
      template < 
        typename T
        , typename bound_function_type
      >
      inline bool blocking_constraint_search( 
      boost::numeric::ublas::vector<T> const & x
      , boost::numeric::ublas::vector<T> const & p
      , bound_function_type const & c
      , T & tau 
      , size_t & blocking_idx
      )
      {
        typedef OpenTissue::math::ValueTraits<T>  value_traits;

        //
        //
        // Say we some problem to be solved for x \in R^n subject
        // to the m constraints
        //
        //   c_i(x) >= 0 ;  i = 1...m
        //
        // Given the current value of the feasible iterate x and a search direction p, we
        // want to determine the largest possible step-length, tau \leq 1, that can
        // be taken such that
        //
        //  c_i(x+tau*p) >= 0 ; i = 1...m
        //
        // For this value of the step-length tau we also want to determine
        // the subset of blocking constraints. A blocking constraint is a constraint
        // that limits the value of the step-length, thus
        //
        //  c_j(x+tau*p) = 0 ; if j is a blocking constraint
        //
        // In general the constraint function c_i(x) could be any function.
        // To simplify matter in our search method we will linearize the constraint
        // function and work with the linear approximation instead. That is,
        //
        //  c_i(x + tau p) = c_i(x)  + \tau \nabla c_i^T p + O(|tau|)
        //           \approx c_i(x)  + \tau \nabla c_i^T p 
        //
        // We know that $\tau \geq 0$ and since x is a feasible point we also
        // know $c_i(x) \geq 0$. This implies that if $\nabla c_i^T p \geq 0$ we
        // have $c_i(x + tau p) \geq 0$. Thus the step-length is unbounded wrt. this
        // constraint. However, if $\nabla c_i^T p<0$ then for $tau$ large enough
        // we will have $c_i(x + \tau p) = 0$. 
        //
        // Using our linearization we can compute an upper bound for the step-length
        // as follows
        //
        //  c_i(x)  + \tau \nabla c_i^T p \geq 0
        //  \tau  \leq \frac{- c_i(x)}{\nabla c_i^T p}
        //
        // Let us iintroduce the symbol tau_i to denote the upper bound of the
        // i'th constraint then
        //
        //    \tau_i  = \frac{- c_i(x)}{\nabla c_i^T p}      ; if \nabla c_i^T p< 0
        //            = \infty                               ; otherwise
        //
        // Now we can determine the maximum possible step length as
        //
        //   tau = min ( 1, tau_1,... tau_m )
        //
        // If tau_i is equal to tau then we have a blocking constraint.
        //
        // Notice that if the constraint function is a linear function
        // then our search is exact if not then our search is only exact
        // to within first-order due to our linearization. Observe that
        // for typical lower and upper bound functions we can define the constraints as
        //
        //  c(x) = x - l(x)\geq 0
        //  c(x) = u(x) - x \geq 0
        //
        // If l and u are constants then we have \nabla c_i = \pm e_i = \pm (0..0 1 0...0)^T. 
        //
        //
        size_t const n = x.size();

        if(n<=0)
          throw std::invalid_argument("x was empty");

        if(n != p.size())
          throw std::invalid_argument("line-search direction was incompatible with the iterate");

        bool is_blocked = false;
        tau = value_traits::one();
        blocking_idx = n + 1;

        for (size_t i = 0; i < n; ++ i)
        {
          T denom = value_traits::zero();
          typename bound_function_type::vector_iterator  e   = c.partial_begin(i);
          typename bound_function_type::vector_iterator  end = c.partial_end(i);
          for(; e!=end; ++e)
            denom += p(e.index())*(*e);

          assert( is_number( denom ) || !"blocking_constraint_search() NAN encounted!");

          if(denom<value_traits::zero())
          {
            T num = -c(x,i);
            assert( is_number( num ) || !"blocking_constraint_search() NAN encounted!");

            T tau_i = num / denom;
            assert( is_number( tau_i ) || !"blocking_constraint_search() NAN encounted!");

            if(tau_i < tau)
            {
              tau = tau_i;
              blocking_idx = i;
              is_blocked = true;
            }
          }

        }
        return is_blocked;
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_BLOCKING_CONSTRAINT_SEARCH_H
#endif
