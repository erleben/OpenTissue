#ifndef OPENTISSUE_CORE_MATH_BIG_CONJUGATE_GRADIENT_H
#define OPENTISSUE_CORE_MATH_BIG_CONJUGATE_GRADIENT_H
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

#include <boost/cast.hpp>             // needed for boost::numeric_cast
#include <stdexcept>
#include <cmath>


namespace OpenTissue
{
  namespace math
  {
    namespace big
    {

      /**
      * Conjugate Gradient Solver.
      *
      * This implementation is based on code from Gunther Winkler.
      * See: http://www-user.tu-chemnitz.de/~wgu/ublas/matrix_sparse_usage.html
      *
      *
      * @param A                 A symmetric positive definite matrix.
      * @param x                 Upon return this argument holds a the solution to the system A x = b
      * @param b                 The right hand side vector.
      * @param max_iterations    The maximum number of iterates allowed.
      * @param epsilon           The stopping threshold to be used.
      * @param iterations        Upon return this argument holds the number of used iterations.
      */
      template<typename matrix_type, typename vector_type>
      inline void conjugate_gradient( 
        matrix_type const & A
        , vector_type & x
        , vector_type const & b
        , size_t const & max_iterations
        , typename vector_type::value_type const & epsilon 
        , size_t & iterations
        )
      {
        typedef typename vector_type::value_type           value_type;
        typedef typename vector_type::size_type            size_type;
        typedef OpenTissue::math::ValueTraits<value_type>  value_traits;

        if(max_iterations <= 0)
          throw std::invalid_argument("Max iterations should be positive" );

        if(epsilon <= value_traits::zero())
          throw std::invalid_argument("epsilon should be positive" );

        if(A.size1() <= 0 || A.size2() <= 0)
          throw std::invalid_argument("A was empty");

        if(b.size() != A.size1())
          throw std::invalid_argument("The size of b must be the same as the number of rows in A");

        if(x.size() != A.size2())
          throw std::invalid_argument("The size of x must be the same as the number of columns in A");

        if(A.size1() != A.size2())
          throw std::invalid_argument("A is not quadratic");

        iterations = 0; ///< If called multiple times, the iteration counter needs to be cleared!

        size_type const size = x.size();

        vector_type r( size ); ///< Residual vector.
        vector_type g( size ); ///< Search direction (gradient).
        vector_type d( size ); ///< The next Kyrlov subspace vector, A*x^{k+1}

        value_type alpha, alpha2, beta, gamma;

        // r = b - prod(A, x);
        ublas::noalias( r ) = b;
        ublas::axpy_prod( A, -x, r, false );

        ublas::noalias( g ) = r;
        alpha2 = ublas::inner_prod( r, r );

        ++iterations;

        value_type const threshold = epsilon * epsilon * alpha2;

        while ( ( iterations < max_iterations ) && ( alpha2 > threshold ) )
        {
          // d = prod(A, g);
          ublas::axpy_prod( A, g, d, true );

          gamma = ublas::inner_prod( g, d );
          alpha = alpha2;

          ublas::noalias( x ) += (  alpha / gamma ) * g;
          ublas::noalias( r ) += ( -alpha / gamma ) * d;

          alpha2 = ublas::inner_prod( r, r );
          beta = alpha2 / alpha;

          ublas::noalias( g ) = r + beta * g;

          ++iterations;
        }       
      }

      /**
      * Conjugate Gradient Solver.
      *
      * @param A                 A symmetric positive definite matrix.
      * @param x                 Upon return this argument holds a the solution to the system A x = b
      * @param b                 The right hand side vector.
      * @param iterations        Upon return this argument holds the number of used iterations.
      */
      template<typename matrix_type, typename vector_type>
      inline void conjugate_gradient( 
        matrix_type const & A
        , vector_type & x
        , vector_type const & b
        , size_t & iterations
        )
      {
        typedef typename vector_type::value_type  value_type;
        value_type epsilon = boost::numeric_cast<value_type>(10e-6);
        conjugate_gradient(A,x,b,15u,epsilon,iterations);
      }

      /**
      * Conjugate Gradient Solver.
      *
      * @param A                 A symmetric positive definite matrix.
      * @param x                 Upon return this argument holds a the solution to the system A x = b
      * @param b                 The right hand side vector.
      */
      template<typename matrix_type, typename vector_type>
      inline void conjugate_gradient( 
        matrix_type const & A
        , vector_type & x
        , vector_type const & b
        )
      {
        typedef typename vector_type::value_type  value_type;
        value_type epsilon = boost::numeric_cast<value_type>(10e-4);
        size_t iterations;
        conjugate_gradient(A,x,b,15u,epsilon,iterations);
      }

      /**
      * Conjugate Gradient Functor.
      * This is a convenience class providing a functor interface for all the free template functions.
      *
      * It is the intention that this will make it easier for
      * end-users to pass the appropriate version of the Conjugate Gradient
      * method to other algorithms.
      */
      class ConjugateGradientFunctor
      {
      public:

        template<typename matrix_type, typename vector_type>
        void operator() ( 
          matrix_type const & A
          , vector_type & x
          , vector_type const & b
          , size_t const & max_iterations
          , typename vector_type::value_type const & epsilon 
          , size_t & iterations
          )
        {
          conjugate_gradient(A,x,b,max_iterations,epsilon,iterations);
        }

        template<typename matrix_type, typename vector_type>
        void operator()( 
          matrix_type const & A
          , vector_type & x
          , vector_type const & b
          )
        {
          conjugate_gradient(A,x,b);
        }

        template<typename matrix_type, typename vector_type>
        void operator()( 
          matrix_type const & A
          , vector_type & x
          , vector_type const & b
          , size_t & iterations
          )
        {
          conjugate_gradient(A,x,b,iterations);
        }

      };

    } // namespace big
  } // namespace math
} // namespace OpenTissue

// OPENTISSUE_CORE_MATH_BIG_SVD_H
#endif
