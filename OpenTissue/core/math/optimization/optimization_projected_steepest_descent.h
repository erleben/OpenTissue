#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_STEEPEST_DESCENT_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_STEEPEST_DESCENT_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_constants.h>
#include <OpenTissue/core/math/optimization/optimization_armijo_projected_backtracking.h>
#include <OpenTissue/core/math/optimization/optimization_stationary_point.h>

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
      * A projected steepest descent method.
      *
      * This implementation was made to solve problems of the type
      *
      * \f[  \vec x^* = \min_{\vec x} f(\vec x)  \text{s.t.}  \vec l \leq x \leq \vec u \f]
      *
      * It does so by peroforming a projected line-searh. The idea is to rewrite
      * the constraints as a projection operator,
      *
      * \f[  P(\vec x) = \min\left( \vec u, \max\left( \vec x, \vec l\right)    \right)   \f]
      *
      * See the file optimization_project for examples of such re-writes. Next the projection
      * operator is applied during the line-search to make sure that only feasible
      * iterates are generated. See the method armijo_projected_backtracking for
      * details on the line-search performed.
      *
      * @param f                     The function that we seek a minimizer of.
      * @param nabla_f               The gradient of the function that we seek a minizer of.
      * @param x                     Upon call this argument holds the current value of
      *                              the iterate (usefull for warmstarting). Upon return
      *                              this argument holds solution found by the method.
      * @param max_iterations        This argument holds the value of the maximum allowed
      *                              iterations.
      * @param absolute_tolerance    This argument holds the value used in the absolute
      *                              stopping criteria. Setting the value to zero will make the test in-effective.
      * @param relative_tolerance    This argument holds the value used in the relative stopping criteria.
      *                              Setting the value to zero will make the test in-effective.
      * @param stagnation_tolerance  This argument holds the value used in the stagnation test. It is
      *                              an upper bound of the infinity-norm of the difference in the x-solution
      *                              between two iterations.  Setting the value to zero will make the test in-effective.
      * @param status                Upon return this argument holds the status of the computation.
      *                              If status = 0 then iterating has not started due some error
      *                              If status = 1 then we have not converged to a solution
      *                              If status = 2 then absolute convergence
      *                              If status = 3 then the method has stagnated ie. there is not
      *                              a sufficient change in x between two iterations.
      *                              If status = 4 then the relative improvement in our merit
      *                              function is too small between two iterations.
      *                              If status = 5 then back-tracking failed to produced a step yielding a decrease
      * @param iteration             Upon return this argument holds the number of the iteration
      *                              when the method exited.
      * @param error                 Upon return this argument holds the value of the error of
      *                              the solution when the method exited.
      * @param alpha                 Armijo test paramter, should be in the range 0..1, this is the fraction of sufficient decrease that is needed by the line-search method. A good value is often 0.00001;
      * @param beta                  The step-length reduction parameter. Everytime the Armijo condition fails then the step length is reduced by this fraction. Usually alpha < beta < 1. A good value is often 0.5;
      * @param profiling             If this argument is null then profiling is off. If the pointer
      *                              is valid then profiling is turned on and upon return the vector
      *                              that is pointed to will hold the values of the merit function
      *                              at the iterates.
      * @param P                     The projection operator to be used. Observe that one
      *                              could specify any projection operator. Even one with
      *                              variable bounds.
      */
      template < 
        typename T
        , typename function_functor
        , typename gradient_functor
        , typename projection_operator
      >
      inline void projected_steepest_descent(
      function_functor  & f
      , gradient_functor & nabla_f
      , boost::numeric::ublas::vector<T> & x
      , projection_operator const & P
      , size_t const & max_iterations
      , T      const & absolute_tolerance
      , T      const & relative_tolerance
      , T      const & stagnation_tolerance
      , size_t       & status
      , size_t       & iteration
      , T            & error
      , T      const & alpha
      , T      const & beta
      , ublas::vector<T> * profiling = 0
      )  
      {
        using std::fabs;
        using std::min;
        using std::max;

        typedef          ublas::compressed_matrix<T>       matrix_type;
        typedef          ublas::vector<T>                  vector_type;
        typedef          T                                 real_type;
        typedef          OpenTissue::math::ValueTraits<T>  value_traits;

        if(max_iterations <= 0)
          throw std::invalid_argument("max_iterations must be larger than zero");
        if(absolute_tolerance < value_traits::zero() )
          throw std::invalid_argument("absolute_tolerance must be non-negative");
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
        if(profiling == &x)
          throw std::logic_error("profiling must not point to x-vector");

        error             = value_traits::infinity();
        iteration = 0;

        status = OK;

        size_t const m = x.size();
        if(m==0)
          return;

        status = ITERATING; // Indicate that we are iterating and have not converged

        if(profiling)
        {
          (*profiling).resize( max_iterations );
          (*profiling).clear();
        }

        // Declare temporary storage
        vector_type dx;
        vector_type x_old;
        vector_type nabla_f_k;

        // Allocate space for temporaries
        dx.resize(m);
        x_old.resize(m);
        nabla_f_k.resize(m);

        // Initialize 

        x = P(x); // Make sure that the initial x-value is a feasible iterate!
        real_type             f_0   = f(x);
        ublas::noalias( nabla_f_k ) = nabla_f(x);

        // Iterate until convergence
        for (; iteration < max_iterations; ++iteration)
        {
          if(profiling)
            (*profiling)(iteration) = f_0;

          // Check for absolute convergence
          if(stationary_point( nabla_f_k, absolute_tolerance, error ) )
          {
            status = ABSOLUTE_CONVERGENCE;
            return;
          }

          // Obtain descent direction
          dx.assign( -nabla_f_k );

          // Perform a projected line-search along the descent direction
          x_old.assign( x );
          real_type f_tau = f_0;
          armijo_projected_backtracking(
            f
            , nabla_f_k
            , x_old
            , x
            , dx
            , relative_tolerance
            , stagnation_tolerance
            , alpha
            , beta
            , f_tau
            , status
            , P
            );

          if(status != OK)
            return;

          // Update values for next iteration
          f_0 = f_tau;
          ublas::noalias( nabla_f_k ) = nabla_f(x);

        }//end for loop
      }



      /**
      * A projected steepest descent method.
      *
      * In this version no line-search is performed, instead a projected steepest
      * descent direction is used to update the current iterate.
      *
      * @param f                     The function that we seek a minimizer of.
      * @param nabla_f               The gradient of the function that we seek a minizer of.
      * @param x                     Upon call this argument holds the current value of
      *                              the iterate (usefull for warmstarting). Upon return
      *                              this argument holds solution found by the method.
      * @param max_iterations        This argument holds the value of the maximum allowed
      *                              iterations.
      * @param absolute_tolerance    This argument holds the value used in the absolute
      *                              stopping criteria. Setting the value to zero will make the test in-effective.
      * @param relative_tolerance    This argument holds the value used in the relative stopping criteria.
      *                              Setting the value to zero will make the test in-effective.
      * @param stagnation_tolerance  This argument holds the value used in the stagnation test. It is
      *                              an upper bound of the infinity-norm of the difference in the x-solution
      *                              between two iterations.  Setting the value to zero will make the test in-effective.
      * @param status                Upon return this argument holds the status of the computation.
      *                              If status = 0 then iterating has not started due some error
      *                              If status = 1 then we have not converged to a solution
      *                              If status = 2 then absolute convergence
      *                              If status = 3 then the method has stagnated ie. there is not
      *                              a sufficient change in x between two iterations.
      *                              If status = 4 then the relative improvement in our merit
      *                              function is too small between two iterations.
      *                              If status = 5 then back-tracking failed to produced a step yielding a decrease
      * @param iteration             Upon return this argument holds the number of the iteration
      *                              when the method exited.
      * @param error                 Upon return this argument holds the value of the error of
      *                              the solution when the method exited.
      * @param tau                   The step-size to be used in each iteration.
      * @param profiling             If this argument is null then profiling is off. If the pointer
      *                              is valid then profiling is turned on and upon return the vector
      *                              that is pointed to will hold the values of the merit function
      *                              at the iterates.
      * @param P                     The projection operator to be used. Observe that one
      *                              could specify any projection operator. Even one with
      *                              variable bounds.
      */
      template < 
        typename T
        , typename function_functor
        , typename gradient_functor
        , typename projection_operator
      >
      inline void projected_steepest_descent(
      function_functor  & f
      , gradient_functor & nabla_f
      , boost::numeric::ublas::vector<T> & x
      , projection_operator const & P
      , size_t const & max_iterations
      , T      const & absolute_tolerance
      , T      const & relative_tolerance
      , T      const & stagnation_tolerance
      , size_t       & status
      , size_t       & iteration
      , T            & error
      , T      const & tau
      , ublas::vector<T> * profiling = 0
      )  
      {
        using std::fabs;
        using std::min;
        using std::max;

        typedef          ublas::compressed_matrix<T>       matrix_type;
        typedef          ublas::vector<T>                  vector_type;
        typedef          T                                 real_type;
        typedef          OpenTissue::math::ValueTraits<T>  value_traits;

        if(max_iterations <= 0)
          throw std::invalid_argument("max_iterations must be larger than zero");
        if(absolute_tolerance < value_traits::zero() )
          throw std::invalid_argument("absolute_tolerance must be non-negative");
        if(relative_tolerance < value_traits::zero() )
          throw std::invalid_argument("relative_tolerance must be non-negative");
        if(stagnation_tolerance < value_traits::zero() )
          throw std::invalid_argument("stagnation_tolerance must be non-negative");
        if(profiling == &x)
          throw std::logic_error("profiling must not point to x-vector");
        if(tau < value_traits::zero() )
          throw std::invalid_argument("step size must be a positive number");

        error             = value_traits::infinity();
        iteration = 0;

        status = OK;

        size_t const m = x.size();
        if(m==0)
          return;

        status = ITERATING; // Indicate that we are iterating and have not converged

        if(profiling)
        {
          (*profiling).resize( max_iterations );
          (*profiling).clear();
        }

        // Declare temporary storage
        vector_type dx;
        vector_type x_new;
        vector_type nabla_f_k;

        // Allocate space for temporaries
        dx.resize(m);
        x_new.resize(m);
        nabla_f_k.resize(m);

        // Initialize 

        x = P(x); // Make sure that the initial x-value is a feasible iterate!
        real_type             f_0   = f(x);
        ublas::noalias( nabla_f_k ) = nabla_f(x);

        // Iterate until convergence
        for (; iteration < max_iterations; ++iteration)
        {
          if(profiling)
            (*profiling)(iteration) = f_0;

          // Check for absolute convergence
          if(stationary_point( nabla_f_k, absolute_tolerance, error ) )
          {
            status = ABSOLUTE_CONVERGENCE;
            return;
          }

          dx.assign( -nabla_f_k );

          x_new = P(x + tau*dx);
          T f_1 = f(x_new);

          assert( is_number( f_1 ) || !"projected_steepest_descent(): internal error, NAN is encountered?");

          T gamma = ublas::inner_prod( nabla_f_k, x_new - x );
          assert( is_number( gamma ) || !"projected_steepest_descent(): internal error, NAN is encountered?");

          // Test if we have non descent direction
          if( gamma >= value_traits::zero() )
          {
            status = NON_DESCENT_DIRECTION;
            f_1 = f_0;
            x_new.assign( x );
          }

          if(stagnation( x, x_new, stagnation_tolerance ) )
          {
            status = STAGNATION;
          }

          if(relative_convergence(f_0,f_1, relative_tolerance) )
          {
            status = RELATIVE_CONVERGENCE;
          }

          x.assign( x_new );
          f_0 = f_1;
          nabla_f_k.assign( nabla_f(x) );

          if(status != ITERATING)
            return;

        }//end for loop
      }


    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_STEEPEST_DESCENT_H
#endif
