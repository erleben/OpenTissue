#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_GAUSS_SEIDEL_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_GAUSS_SEIDEL_H
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
#include <OpenTissue/core/math/optimization/optimization_relative_convergence.h>
#include <OpenTissue/core/math/optimization/optimization_absolute_convergence.h>
#include <OpenTissue/core/math/big/big_prod_row.h>

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
      * Project Gauss Seidel Solver for Mixed Complementarity Problems.
      *
      * Given A and b, let y = A x + b and l(x) <= 0 <= u(x). Then this method computes
      * a x-solution to the mixed complimentarity problem defined by:
      *
      *   if  y_i > 0 then x_i = l_i(x) 
      *   if  y_i < 0 then x_i = u_i(x) 
      *   if l_i(x) < x < u_i then y=0
      *
      * During iterations the nonsmooth reformulation
      *
      *   H(x) = max( x -u(x), min( x-l(x), A x + b ) ) 
      *
      * And the merit function
      *
      *  theta(x) = H(x)^T H(x) * 1/2
      *
      * Is used to measure convergence.
      *
      * @param A                     The coefficient matrix. 
      * @param b                     The b-vector.
      * @param l                     A bound function object type. This object represents the lower bounds and must implement the method with the signature:
      *
      *                                  T const & () (ublas::vector<T> const & x, size_t const & i) const 
      *
      * @param u                     A bound function object type (same type as l-parameter). This object represents the upper bounds.
      *
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
      * @param iteration             Upon return this argument holds the number of the iteration
      *                              when the method exited.
      * @param accuracy              Upon return this argument holds the value of the accuracy of
      *                              the solution when the method exited.
      * @param relative_accuracy     Upon return this argument holds the value of the relative accuracy
      *                              between the last two iterations of the method.
      * @param profiling             If this argument is null then profiling is off. If the pointer
      *                              is valid then profiling is turned on and upon return the vector
      *                              that is pointed to will hold the values of the merit function
      *                              at the iterates.
      */
      template < typename T, typename bound_function_type>
      inline void projected_gauss_seidel(
        ublas::compressed_matrix<T> const & A
        , ublas::vector<T> const & b
        , bound_function_type const & l
        , bound_function_type const & u
        , boost::numeric::ublas::vector<T> & x
        , size_t const & max_iterations
        , T      const & absolute_tolerance
        , T      const & relative_tolerance
        , T      const & stagnation_tolerance
        , size_t       & status
        , size_t       & iteration
        , T            & accuracy
        , T            & relative_accuracy
        , ublas::vector<T> * profiling = 0
        )  
      {
        using std::fabs;
        using std::min;
        using std::max;

        typedef          ublas::vector<T>                  vector_type;
        typedef typename vector_type::size_type            size_type;
        typedef          T                                 real_type;
        typedef          OpenTissue::math::ValueTraits<T>  value_traits;

        static real_type const one_half = value_traits::one()/value_traits::two();

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

        if(profiling == &b)
          throw std::logic_error("profiling must not point to b-vector");

        accuracy             = value_traits::infinity();
        relative_accuracy    = value_traits::infinity();
        iteration = 0;
        status = OK;

        size_type m = b.size();

        if(m==0)
          return;

        if(x.size()<=0)
          throw std::invalid_argument("size of x-vector were zero");

        if(profiling)
        {
          (*profiling).resize( max_iterations );
          (*profiling).clear();
        }

        // Create a boolean flag indicating whether we need to compute
        // the value of the merit function or not.
        bool const compute_merit = profiling || (absolute_tolerance > value_traits::zero()) || (absolute_tolerance > value_traits::zero());

        status = ITERATING;

        for ( ;iteration < max_iterations;  )
        {
          ++iteration;

          real_type max_dx = value_traits::zero();

          for (size_type i = 0; i < m; ++ i)
          {
            real_type l_i = l(x,i);
            real_type u_i = u(x,i);

            assert(l_i<= value_traits::zero()  || !"projected_gauss_seidel: lower limit was positive");
            assert(u_i>= value_traits::zero()  || !"projected_gauss_seidel: upper limit was negative");

            real_type y_i = b(i) + OpenTissue::math::big::prod_row(A,x,i);

            real_type const A_ii = A(i,i);

            assert(A_ii> value_traits::zero() || A_ii<value_traits::zero() || !"projected_gauss_seidel: diagonal entry is zero?");

            real_type const old_x = x(i);
            real_type const x_tmp = - (y_i / A_ii) + old_x;

            assert(is_number(x_tmp) || !"projected_gauss_seidel: not a number encountered");

            real_type const x_i =  (x_tmp < l_i) ? l_i :  ( (x_tmp > u_i) ? u_i : x_tmp );

            assert(is_number(x_i) || !"projected_gauss_seidel: not a number encountered");

            x(i) = x_i;

            max_dx = max( max_dx, fabs( x_i - old_x ) );
          }

          // Check for stagnation
          if(max_dx < stagnation_tolerance) 
          {
            status = STAGNATION;
            return;
          }

          // Compute new merit value and perform check stopping criteria's
          if( compute_merit )
          {
            real_type const old_accuracy = accuracy;
            accuracy = value_traits::zero();
            for (size_type i = 0; i < m; ++ i)
            {
              real_type const l_i = l(x,i);
              real_type const u_i = u(x,i);
              real_type const y_i = b(i) + OpenTissue::math::big::prod_row(A,x,i);
              real_type const x_i = x(i);
              real_type const h_i = max( x_i - u_i, min( x_i - l_i, y_i ) );
              accuracy += h_i*h_i;
            }
            accuracy *= one_half;

            if(profiling)
              (*profiling)(iteration-1) = accuracy;

            if( absolute_convergence( accuracy, absolute_tolerance) )
            {
              status = ABSOLUTE_CONVERGENCE;
              return;
            }

            if( relative_convergence(old_accuracy, accuracy, relative_tolerance) )
            {
              status = RELATIVE_CONVERGENCE;
              return;
            }
          }
        }
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_PROJECTED_GAUSS_SEIDEL_H
#endif
