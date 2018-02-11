#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/big/big_types.h>
#include <OpenTissue/core/math/big/big_prod_add_rhs.h>
#include <OpenTissue/core/math/big/big_shur_system.h>
#include <OpenTissue/core/math/big/big_prod_trans.h>

#include <OpenTissue/core/math/optimization/non_smooth_newton/optimization_compute_inverse_D.h>
#include <OpenTissue/core/math/optimization/non_smooth_newton/optimization_compute_jacobian.h>
#include <OpenTissue/core/math/optimization/non_smooth_newton/optimization_compute_partitioned_jacobian.h>
#include <OpenTissue/core/math/optimization/optimization_compute_generalized_minimal_map.h>

#include <OpenTissue/core/math/optimization/optimization_armijo_backtracking.h>
#include <OpenTissue/core/math/optimization/optimization_compute_natural_merit.h>
#include <OpenTissue/core/math/optimization/optimization_compute_index_reordering.h>
#include <OpenTissue/core/math/optimization/optimization_compute_index_sets.h>
#include <OpenTissue/core/math/optimization/optimization_agglomerate_vector.h>
#include <OpenTissue/core/math/optimization/optimization_partition_vector.h>
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

      namespace detail
      {

        template<typename T, typename bound_function_type>
        class ThetaFunctor
        {
        protected:

          ublas::compressed_matrix<T> const & m_A;
          ublas::vector<T> const & m_b;
          bound_function_type const & m_l;
          bound_function_type const & m_u;
          ublas::vector<T> & m_H;
          ublas::vector<T> & m_y;

        public:

          ThetaFunctor(
            ublas::compressed_matrix<T> const & A
            , ublas::vector<T> const & b
            , bound_function_type const & l
            , bound_function_type const & u
            , ublas::vector<T> & H
            , ublas::vector<T> & y
            )
            : m_A(A)
            , m_b(b)
            , m_l(l)
            , m_u(u)
            , m_H(H)
            , m_y(y)
          {}

          T operator()( ublas::vector<T> const & x ) const
          {
            OpenTissue::math::big::prod_add_rhs(m_A,x,m_b,m_y);
            compute_generalized_minimal_map(m_y,m_l,m_u,x,m_H);
            return compute_natural_merit(m_H);
          }

        };

        template<typename T, typename bound_function_type>
        class NablaThetaFunctor
        {
        protected:

          ublas::compressed_matrix<T> const & m_A;
          ublas::vector<T> const & m_b;
          bound_function_type const & m_l;
          bound_function_type const & m_u;
          ublas::vector<T> & m_H;
          ublas::vector<T> & m_y;

        public:

          NablaThetaFunctor(
            ublas::compressed_matrix<T> const & A
            , ublas::vector<T> const & b
            , bound_function_type const & l
            , bound_function_type const & u
            , ublas::vector<T> & H
            , ublas::vector<T> & y
            )
            : m_A(A)
            , m_b(b)
            , m_l(l)
            , m_u(u)
            , m_H(H)
            , m_y(y)
          {}

          ublas::vector<T> operator()( ublas::vector<T> const & x )
          {
            OpenTissue::math::big::prod_add_rhs(m_A,x,m_b,m_y);
            compute_generalized_minimal_map(m_y,m_l,m_u,x,m_H);

            ublas::vector<size_t> bitmask;
            size_t cnt_active = 0;
            size_t cnt_inactive = 0;

            compute_index_sets( m_y, x, m_l, m_u, bitmask, cnt_active, cnt_inactive );

            ublas::compressed_matrix<T> J;
            detail::compute_jacobian( m_A, m_l, m_u, bitmask, J );

            ublas::vector<T> nabla_theta;
            nabla_theta.resize(x.size(), false);

            OpenTissue::math::big::prod_trans(J, m_H, nabla_theta);

            return nabla_theta;
          }

        };


      } // namespace detail 

      /**
      * Solve Mixed Complimentarity Problems using a Non-smooth Newton Method.
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
      *                              If status = 5 then back-tracking failed to produced a step yielding a decrease
      * @param iteration             Upon return this argument holds the number of the iteration
      *                              when the method exited.
      * @param accuracy              Upon return this argument holds the value of the accuracy of
      *                              the solution when the method exited.
      * @param alpha                 Armijo test paramter, should be in the range 0..1, this is the fraction of sufficient decrease that is needed by the line-search method. A good value is often 0.00001;
      * @param beta                  The step-length reduction parameter. Everytime the Armijo condition fails then the step length is reduced by this fraction. Usually alpha < beta < 1. A good value is often 0.5;
      * @param sub_system_solver     The solver that is to be used to solve the linear sub-system. If an iterative solver is used then the relative residual must be less than one in order for the NSN method to achieve local convergence.
      * @param use_shur              A boolean flag used to indicate whether the NSN method should
      *                              try to use a shur complement to reduce the linear subsystem into
      *                              a smaller system. It is implicitly assumed that some of the
      *                              sub-matrices have a particular nice pattern to be easily invertible.
      *                              Default value is set to true. If set to false then a ``full'' jacobian
      *                              matrix is used instead, and in this case there is no requirements on
      *                              the shape of the Jacobian matrix.
      * @param profiling             If this argument is null then profiling is off. If the pointer
      *                              is valid then profiling is turned on and upon return the vector
      *                              that is pointed to will hold the values of the merit function
      *                              at the iterates.
      */
      template < typename T, typename bound_function_type, typename solver_function_type>
      inline void non_smooth_newton(
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
        , T      const & alpha
        , T      const & beta
        , solver_function_type const & sub_system_solver
        , bool const & use_shur = true
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
        if(profiling == &b)
          throw std::logic_error("profiling must not point to b-vector");

        status = OK;

        size_t const m = b.size();
        if(m==0)
          return;

        if(x.size()<=0)
          throw std::invalid_argument("size of x-vector were zero");

        // Temporary storage used by NSN method. 
        vector_type rhs(m);
        vector_type H(m);
        vector_type dx(m);
        vector_type x_old(m);
        vector_type y(m);
        ublas::vector<size_t> bitmask;
        ublas::vector<size_t> old2new;
        ublas::vector<size_t> new2old;
        matrix_type A_aa;
        matrix_type A_ab;
        matrix_type C;
        matrix_type D;
        vector_type rhs_a;
        vector_type rhs_b;
        vector_type dx_a;
        vector_type dx_b;
        matrix_type J;

        detail::ThetaFunctor<real_type,bound_function_type> theta(A, b, l, u, H, y);
        detail::NablaThetaFunctor<real_type,bound_function_type> nabla_theta(A, b, l, u, H, y);

        // If warm-started we must compute the accuracy of the initial guess
        accuracy             = theta(x);
        iteration            = 0;
        status               = ITERATING;

        if(profiling)
        {
          (*profiling).resize( max_iterations );
          (*profiling).clear();
        }
        for (; iteration < max_iterations; )
        {
          if(profiling)
            (*profiling)(iteration) = accuracy;
          ++iteration;

          if( absolute_convergence( accuracy, absolute_tolerance) )
          {
            status = ABSOLUTE_CONVERGENCE;
            return;
          }

          ublas::noalias(rhs) = -H;
          size_t cnt_active = 0;
          size_t cnt_inactive = 0;
          compute_index_sets( y, x, l, u, bitmask, cnt_active, cnt_inactive );
          if(use_shur)
          {
            compute_index_reordering( bitmask, old2new, new2old );
            detail::compute_partitioned_jacobian( A, l, u,bitmask, old2new, cnt_active, cnt_inactive, A_aa, A_ab, C, D);
            partition_vector( rhs, bitmask, old2new, cnt_active, cnt_inactive, rhs_a, rhs_b );

            // solve partitioned Newton Subsystem using a Shur Complement
            //
            //   | A_aa  A_ab | | dx_a | = | rhs_a |
            //   | C     D    | | dx_b |   | rhs_b |
            //
            // Compute D^{-1} (flip sign on lower part of D),
            detail::compute_inverse_D( D );

            // Note: From Ricky's proof: The relative residual must be less
            // than one in order for the approximate solution dx to be a descent
            // direction for the merit function
            OpenTissue::math::big::shur_system( A_aa, A_ab, C, D, rhs_a, rhs_b, dx_a, dx_b, sub_system_solver);
            agglomerate_vector(dx_a, dx_b, new2old, dx);
          }          
          else
          {
            detail::compute_jacobian( A, l, u, bitmask, J );
            sub_system_solver( J, dx, rhs );
          }

          vector_type grad = nabla_theta(x);
          x_old = x;

          armijo_backtracking(
            theta
            , grad
            , x_old
            , x
            , dx
            , relative_tolerance
            , stagnation_tolerance
            , alpha
            , beta
            , accuracy
            , status
            );

          if( status != OK )
            return;
          status = ITERATING;

        }
      }

    } // namespace optimization
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_OPTIMIZATION_NON_SMOOTH_NEWTON_H
#endif
