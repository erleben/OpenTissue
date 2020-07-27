#ifndef OPENTISSUE_CORE_MATH_OPTIMIZATION_BFGS_H
#define OPENTISSUE_CORE_MATH_OPTIMIZATION_BFGS_H
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
#include <OpenTissue/core/math/optimization/optimization_armijo_backtracking.h>
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

      namespace detail
      {

        /**
        * Inverse Hessian Update Formula.
        *
        * H_k = B_k^{-1}, where B_k is the Hessian approximation in the k'th step.
        *
        * The incremental update formula for the inverse Hessian is
        *
        * \f[H_{k+1} = (I - \rho s y^T) H_k (I - \rho y s^T) +   \rho s s^T\f]
        *
        */
        template<typename vector_type,typename matrix_type>
        inline void bfgs_update_inverse_hessian(vector_type const & y,vector_type const & s, matrix_type & H)
        {
          using std::fabs;

          typedef typename matrix_type::value_type                     real_type;
          typedef typename OpenTissue::math::ValueTraits<real_type>    value_traits;

          if(H.size1() <= 0 || H.size2() <= 0)
            throw std::invalid_argument("bfgs_update_inverse_hessian(): Hessian was empty");
          if(y.size() <= 0 )
            throw std::invalid_argument("bfgs_update_inverse_hessian(): y_i was empty");
          if(s.size() <= 0 )
            throw std::invalid_argument("bfgs_update_inverse_hessian(): s_i was empty");

          assert(y.size()==H.size2() || "bfgs_update_inverse_hessian(): inconsistent matrix vector dimensions");

          assert(H.size1()==H.size2() || "bfgs_update_inverse_hessian(): Incompatible dimensions of Hessian matrix");

          size_t const N = H.size1();

          real_type dot = inner_prod(y,s);
          if(  fabs(dot) <= value_traits::zero() ) // Make sure we do not divide by zero
          {            
            dot = value_traits::one();
          }

          real_type rho = value_traits::one() / dot;
          
          //matrix_type A,B,C;          
          //A.resize(N,N,false);
          //B.resize(N,N,false);
          //C.resize(N,N,false);
          //identity_matrix_type I(N,N);
          //A = I - outer_prod(s,y)*rho;
          //B = I - outer_prod(y,s)*rho;
          //C = outer_prod(s,s)*rho;
          //matrix_type tmp;
          //tmp.resize(N,N,false);
          ////ublas::noalias(tmp) = ublas::prod(H,B);
          ////ublas::noalias(H)   = ublas::prod(A,tmp) + C;
          //ublas::noalias(tmp) = ublas::sparse_prod<matrix_type>(H,B);
          //ublas::noalias(H)   = ublas::sparse_prod<matrix_type>(A,tmp) + C;

          //
          //The update formula looks like this
          //
          //\f[H' = (I - rho s y^T) H(I - rho  y s^T) + rho s s^T\f]
          //
          //doing some manipulation one gets
          //
          //\f[H' = H + rho^2  (y^T H y) s s^T + rho s s^T - rho s y^T H   - rho H  y s^T\f]
          //
          //\f$H\f$ is symmetric postive definite so \f$(H y)^T = y^Y H^T = y^T H\f$ and
          //
          //\f[H' = H + (\rho^2  (y^T H y)+\rho) s s^T  - \rho ( s (H y)^T   +  (H  y)  s^T )\f]
          //
          //Introducing \f$v = H y\f$ and \f$\gamma = (\rho^2  (y^T H y)+rho)\f$
          //(note \f$y^T H y = y^T v\f$) this reads
          //
          //\f[H' = H + \gamma (s s^T)  - \rho ( s v^T   + v s^T )\f]
          //
          //Doing one more re-write (just a matter of parentheses) one has
          //
          //\f[H' = H + (\gamma s - \rho v) s^T - (\rho s) v^T \f]
          //
          //This can be efficiently implemented as a succesion of two rank-1 updates
          //
          vector_type v;
          v.resize(N,false);
          ublas::axpy_prod(H,y,v,true); // v = H y
          real_type gamma = rho*rho*ublas::inner_prod(y,v) + rho;
          for (typename vector_type::size_type i = 0; i < N; ++ i)
          {
            ublas::row(H,i) -= rho*s(i) * v;
            ublas::row(H,i) += (gamma*s(i) - rho * v(i)) * s;
          } 

        }

      }

      /**
      * The BFGS Method.
      * The BFGS method implemented here is a damped quassi Newton Method.
      * The implementation is a quassi Newton method due to the fact that
      * an approximation is used for the inverse Hessian matrix. The approximation
      * is incrementally updated in each step of the Newton method. An in-exact
      * line-search method is used. The line-search is based on back-tracking until
      * a sufficient decrease condition (The Armijo Condition) is fulfilled. Because
      * the back-tracking may yield a step-length shorter than the pure Newton
      * step length (\f$\tau = 1\f$) the implemented method is a damped Newton method.
      *
      * Classically the Newton method is used for unconstrained minimization problems.
      * In the following we will derive the Newton method for this problem. Given a
      * twice continuously differentiable real-valued function, \f$f(\vec x) : \Re^n \mapsto
      * \Re\f$, we wish to find a minimizer of the function. That is we want to solve
      * the problem
      * 
      * \f[  \vec x^* = \min_{\vec x} f(\vec x)  \f]
      * 
      * Newtons method is a iterative method and it generates a sequence of iterates,
      * \f$\vec x_0\f$, \f$\vec x_1\f$, \f$\vec x_2\f$, \f$\ldots\f$, \f$\vec x_k\f$. Hoping that the
      * sequence is converging to the minimizer \f$\vec x^*\f$. The \f$k+1\f$ iterate \f$\vec
      * x_{k+1}\f$ are computed using the update formula
      * 
      * \f[  \vec x_{k+1} = \vec x_{k} + \vec dx_k, \f]
      * 
      * here \f$\vec dx_k\f$ is called the Newton direction of the \f$k\f$th step. In the
      * following we will derive the algorithm to find \f$\vec dx_k\f$.
      *
      * From Taylors theorem we have
      *
      * \f[
      *  f(\vec x_k + \vec dx_k) 
      *  = 
      *  f(\vec x_k) 
      *  + 
      *  \frac{1}{2} \nabla f(\vec x_k) \vec dx_k 
      *  + 
      *  \vec dx_k^T \nabla^2 f(\vec x_k ) \vec dx_k 
      *  + 
      *  O(\norm{\vec dx_k}^3),
      *\f]
      *
      *Next we define a local model of the \f$f\f$-function, that is
      *
      *\f[
      *  f(\vec x_k + \vec dx_k)  
      *  \approx   
      *  f(\vec x_k) 
      *  + 
      *  \nabla f(\vec x_k) \vec dx_k 
      *  + 
      *  \frac{1}{2} \vec dx_k^T \nabla^2 f(\vec x_k ) \vec dx_k  
      *  \equiv 
      *  m_k(\vec dx_k)
      *\f]
      *
      * To find the direction that minimizes the \f$m_k\f$-function we use the first order
      * necessary conditions and solves for \f$\vec dx_k\f$ such that the first order
      * derivative of \f$m_k\f$ is zero. That is
      *
      * \f[
      *  \nabla m_k(\vec dx_k) = 0 
      * \f]
      *
      * By straight forward differentiation of \f$m_k\f$ we obtain
      *
      * \f[
      *  \nabla m_k(\vec dx_k) =  \nabla f(\vec x_k)  +  \nabla^2 f(\vec x_k ) \vec dx_k
      * \f]
      *
      * From all this we obtain the so-called Newton equation
      *
      * \f[
      *    \nabla^2 f(\vec x_k ) \vec dx_k  = - \nabla f(\vec x_k)
      * \f]
      *
      * and we use this to solve for the Newton direction, \f$\vec dx_k\f$. Provided that
      * \f$\nabla^2 f(\vec x_k )\f$ is non-singular and always positive-definite one can
      * ensure the Newton method behaves nicely.
      *
      * The next extention is to perform a line-search along the Newton direction. That
      * is we want to determine a step-length, \f$\tau\f$, such that the Newton update,
      *
      * \f[  \vec x_{k+1} = \vec x_{k} + \tau \vec dx_k, \f]
      *
      * results in a sufficient decrease. See the method armijo_backtracking for more
      * details on this.
      *
      * Lastly instead of computing \f$\nabla^2 f(x^k)^{-1}\f$ an approximating matrix is used,
      *
      * \f[  H_k \approx  \nabla^2 f(x^k)^{-1} \f]
      *
      * See the method bfgs_update_inverse_hessian for details.
      *
      *
      * @param f                     The function that we seek a minimizer of.
      * @param nabla_f               The gradient of the function that we seek a minizer of.
      * @param H                     An initial approximation of the inverse Hessian matrix.
      *                              This should be a symmetric positive definite matrix.
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
      */
      template < 
        typename T
        , typename function_functor
        , typename gradient_functor
      >
      inline void bfgs(
      function_functor  & f
      , gradient_functor & nabla_f
      , ublas::compressed_matrix<T>  & H
      , boost::numeric::ublas::vector<T> & x
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

        error     = value_traits::infinity();
        iteration = 0;
        status    = OK;

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
          armijo_backtracking(
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
            );

          if(status != OK)
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

// OPENTISSUE_CORE_MATH_OPTIMIZATION_BFGS_H
#endif
