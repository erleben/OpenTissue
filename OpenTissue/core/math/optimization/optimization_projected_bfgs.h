#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_BFGS_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_BFGS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/optimization/optimization_bfgs.h> // The unprojected version
#include <OpenTissue/core/math/optimization/optimization_constants.h>
#include <OpenTissue/core/math/optimization/optimization_armijo_projected_backtracking.h>
#include <OpenTissue/core/math/optimization/optimization_stationary_point.h>
#include <OpenTissue/core/math/optimization/optimization_absolute_convergence.h>

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
      * A projected BFGS implemention.
      * See the comments for the bfgs method.
      * This implementation was made to solve problems of the type
      *
      * \f[  \vec x^* = \min_{\vec x} f(\vec x)  \text{s.t.}  \vec l \leq x \leq \vec u \f]
      *
      * It does so by performing a projected line-searh. The idea is to rewrite
      * the constraints as a projection operator,
      *
      * \f[  P(\vec x) = \min\left( \vec u, \max\left( \vec x, \vec l\right)    \right)   \f]
      *
      * See the file optimization_project for examples of such re-writes. Next the projection
      * operator is applied during the line-search to make sure that only feasible
      * iterates are generated. See the method armijo_projected_backtracking for
      * details on the line-search performed.
      *
      * @param P      The projection operator to be used. Observe that one
      *               could specify any projection operator. Even one with
      *               variable bounds.
      */
      template < 
        typename T
        , typename function_functor
        , typename gradient_functor
        , typename projection_operator
      >
      inline void projected_bfgs(
        function_functor  & f
      , gradient_functor & nabla_f
      , ublas::compressed_matrix<T>  & H
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
        vector_type y_k;
        vector_type s_k;
        vector_type dx;
        vector_type x_old;
        vector_type nabla_f_k;
        vector_type nabla_f_k1;

        // Allocate space for temporaries
        y_k.resize(m);
        s_k.resize(m);
        dx.resize(m);
        x_old.resize(m);
        nabla_f_k.resize(m);
        nabla_f_k1.resize(m);

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

          // solve Newton system. Compute Search Direction
          // ublas::noalias( dx ) = - ublas::prod(H, nabla_f_k);
          ublas::axpy_prod(H, -nabla_f_k, dx, true);

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
        
          if(status != OK ) 
            return;
                    
          // Do the incremental update of the inverse Hessian approximation
          ublas::noalias( nabla_f_k1 ) = nabla_f(x);
          ublas::noalias( s_k )  = x - x_old;
          ublas::noalias( y_k ) = nabla_f_k1 - nabla_f_k;

          detail::bfgs_update_inverse_hessian(y_k,s_k,H);

          // Update values for next iteration
          f_0 = f_tau;
          nabla_f_k.assign( nabla_f_k1 );

        }//end for loop
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_BFGS_H
#endif
